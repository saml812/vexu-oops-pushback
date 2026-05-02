#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 100;
const int TURN_SPEED = 90;
const int SWING_SPEED = 80;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  // 20.0, 0.0, 100.0
  // 2nd bot
  //   18.30, 0.0, 57.50;        // Fwd/rev constants, used for odom and non odom motions
  //   11.0, 0.0, 20.0        // Holds the robot straight while going forward without odom
  //   4.75, 0.0, 26.75, 0.0     // Turn in place constants
  chassis.pid_drive_constants_set(17.80, 0.0, 97.50);        // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(16.0, 0.0, 22.00);       // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(7.35, 0.0, 37.75, 0.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(5_in, 50);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();

  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

void skillsRight() {
  chassis.pid_turn_behavior_set(ez::shortest);
  chassis.drive_angle_set(0);

  chassis.pid_drive_set(33_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  robotactions::setMatchLoaderUp(false);
  robotactions::setBasketUp(true);
  robotactions::setHoodUp(true);

  chassis.pid_drive_set(7_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(2_in);
  robotactions::spinIntake(robotactions::kFullPowerMv);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-5_in, 40);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5_in, 40);
  pros::delay(1500);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-32_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(200);

  robotactions::stopIntake();
  //   robotactions::setLeverPistonUp(true);
  robotactions::spinLever(robotactions::kFullPowerMv);
  pros::delay(500);

  //   robotactions::setLeverPistonUp(false);
  robotactions::spinLever(-robotactions::kFullPowerMv / 4);
  pros::delay(500);
  robotactions::stopLever();

  chassis.pid_drive_set(18_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-60_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-22_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();

  robotactions::setMatchLoaderUp(true);
  robotactions::setBasketUp(true);
  robotactions::setHoodUp(true);

  chassis.pid_drive_set(13_in, DRIVE_SPEED);
  chassis.pid_wait_until(2_in);
  robotactions::spinIntake(robotactions::kFullPowerMv);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-2_in, 40);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(2_in, 40);
  pros::delay(1500);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-33_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(200);

  robotactions::stopIntake();
  //   robotactions::setLeverPistonUp(true);
  robotactions::spinLever(robotactions::kFullPowerMv);
  pros::delay(100);

  //   robotactions::setLeverPistonUp(false);
  robotactions::spinLever(-robotactions::kFullPowerMv / 4);
  pros::delay(100);
  robotactions::stopLever();

  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  chassis.pid_wait();

  robotactions::setBasketUp(false);

  chassis.pid_turn_set(335_deg, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(12_in, 80);
  chassis.pid_wait_until(8_in);
  robotactions::spinIntake(robotactions::kFullPowerMv);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(ez::RIGHT_SWING, 20_deg, SWING_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(4_in, 30);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 50);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(19_in, 25);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, 70);
  chassis.pid_wait();

  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(28_in, 70);
  chassis.pid_wait();

  chassis.pid_turn_set(225_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(15_in, 70);
  chassis.pid_wait();

  // Low goal
  robotactions::setBasketUp(true);
  robotactions::setIntakeLiftUp(true);
  robotactions::spinIntake(robotactions::kLowGoalMv / 2);
  pros::delay(2000);

  robotactions::setIntakeLiftUp(false);
  robotactions::stopIntake();

  chassis.pid_drive_set(-20_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20_in, 10, true);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 180, SWING_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(17_in, DRIVE_SPEED);
  chassis.pid_wait();

  robotactions::stopIntake();
  robotactions::stopLever();
  robotactions::setBasketUp(false);
  //   robotactions::setLeverPistonUp(false);
  robotactions::setHoodUp(false);
}

void autonRight() {
  chassis.pid_turn_behavior_set(ez::shortest);
  robotactions::setMatchLoaderUp(true);
  chassis.drive_angle_set(125_deg);

  chassis.pid_drive_set(38_in, DRIVE_SPEED);
  chassis.pid_wait_until(10_in);
  robotactions::spinIntake(robotactions::kFullPowerMv);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20_in, 40);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-22_in, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, SWING_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6_in, 70);

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-21_in, 70);
  chassis.pid_wait();
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();

  robotactions::setMatchLoaderUp(false);
  robotactions::setBasketUp(true);
  robotactions::setHoodUp(true);

  chassis.pid_drive_set(12_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10_in, 20);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-1_in, 20);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(1_in, 20);
  pros::delay(800);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-32_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(200);

  robotactions::stopIntake();
  //   robotactions::setLeverPistonUp(true);
  robotactions::spinLever(robotactions::kFullPowerMv);
  pros::delay(100);

  //   robotactions::setLeverPistonUp(false);
  robotactions::spinLever(-robotactions::kFullPowerMv / 4);
  pros::delay(100);

  robotactions::spinLever(robotactions::kFullPowerMv);
  pros::delay(200);

  robotactions::spinLever(-robotactions::kFullPowerMv / 4);
  pros::delay(100);

  robotactions::stopLever();
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true);
  chassis.pid_drive_set(15_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(135_deg, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(-19_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(270, 90);
  chassis.pid_wait();

  chassis.pid_drive_set(-5_in, 110);
  chassis.pid_wait_quick_chain();
  robotactions::setWingUp(false, wingR);
  chassis.pid_drive_set(-18_in, 110);
}

void autonRight1() {
  chassis.pid_turn_behavior_set(ez::shortest);
  robotactions::setMatchLoaderUp(true);
  robotactions::spinLever(-robotactions::kFullPowerMv / 2);
  chassis.drive_angle_set(125_deg);

  chassis.pid_drive_set(38_in, DRIVE_SPEED);
  chassis.pid_wait_until(10_in);
  robotactions::spinIntake(robotactions::kFullPowerMv);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(22_in, 30);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, 100);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 270_deg, 127, 5, ez::longest);
  robotactions::setBasketUp(true);
  robotactions::setHoodUp(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-10_in, 50);
  chassis.pid_wait();

  pros::delay(500);
  robotactions::spinLever(12000);
  pros::delay(500);
  robotactions::spinLever(-robotactions::kFullPowerMv / 2);
  pros::delay(300);
  robotactions::spinLever(12000);
  pros::delay(500);
  robotactions::spinLever(-robotactions::kFullPowerMv / 2);
  pros::delay(300);
  robotactions::stopLever();

  robotactions::spinIntake(9500);

  //   chassis.drive_angle_set(270_deg);
  //   robotactions::spinIntake(robotactions::kFullPowerMv);
  robotactions::setMatchLoaderUp(false);

  chassis.pid_turn_set(270_deg, 80);
  chassis.pid_drive_set(20_in, DRIVE_SPEED);
  robotactions::setHoodUp(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(15_in, 60);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_relative_set(2, 100);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_relative_set(-2, 100);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(270_deg, 80);
  pros::delay(1500);
  robotactions::stopIntake();
  chassis.pid_wait();

  chassis.pid_drive_set(-8_in, 80);
  chassis.pid_wait();
  robotactions::setMatchLoaderUp(true);
  pros::delay(100);
  chassis.pid_turn_set(180_deg, 80);
  chassis.pid_wait();
  robotactions::setIntakeLiftUp(true);
  robotactions::spinIntake(-3000);
  pros::delay(400);
  robotactions::stopIntake();
  robotactions::setIntakeLiftUp(false);

  chassis.pid_turn_set(270_deg, 80);
  chassis.pid_wait();
  robotactions::setMatchLoaderUp(false);
  pros::delay(300);
  chassis.pid_drive_set(10_in, 60);
  chassis.pid_wait_until(5_in);
  robotactions::spinIntake(robotactions::kFullPowerMv);
  pros::delay(2000);
  robotactions::stopIntake();
  chassis.pid_drive_set(-12_in, 40);
  chassis.pid_wait();
  robotactions::setMatchLoaderUp(true);

  chassis.pid_turn_set(45_deg, 80);
  chassis.pid_wait();
  chassis.pid_drive_set(50_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20_in, 65);
  chassis.pid_wait();
  //   robotactions::setIntakeLiftUp(true);
  robotactions::spinIntake(-2500);
  robotactions::setBasketUp(false);
  pros::delay(300);
  robotactions::setBasketUp(true);
  pros::delay(4000);
  chassis.pid_drive_set(-5_in, 40);
  chassis.pid_wait();
  chassis.pid_drive_set(5_in, 40);

  robotactions::stopIntake();
  robotactions::setIntakeLiftUp(false);
}

void autonRight2() {
  chassis.pid_turn_behavior_set(ez::shortest);
  robotactions::setMatchLoaderUp(true);
  robotactions::spinLever(-robotactions::kFullPowerMv / 2);
  robotactions::setIntakeLiftUp(false);
  chassis.drive_angle_set(125_deg);

  //   chassis.pid_drive_set(40_in, DRIVE_SPEED);
  chassis.pid_drive_set(38_in, DRIVE_SPEED);
  chassis.pid_wait_until(10_in);
  robotactions::spinIntake(12000);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(22_in, 30);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, 100);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 270_deg, 127, 3, ez::longest);
  robotactions::setBasketUp(true);
  robotactions::setHoodUp(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-10_in, 50);
  chassis.pid_wait();

  pros::delay(1000);
  robotactions::spinLever(12000);
  pros::delay(500);
  robotactions::spinLever(-robotactions::kFullPowerMv / 2);
  pros::delay(300);
  robotactions::spinLever(12000);
  pros::delay(500);
  robotactions::spinLever(-robotactions::kFullPowerMv / 2);
  pros::delay(300);
  robotactions::stopLever();
  robotactions::setMatchLoaderUp(false);
  robotactions::spinIntake(9500);

  chassis.pid_turn_set(270_deg, 80);
  chassis.pid_drive_set(28_in, DRIVE_SPEED);
  robotactions::setHoodUp(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20_in, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_relative_set(2, 100);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_relative_set(-2, 100);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(270_deg, 80);
  pros::delay(2000);
  //   robotactions::stopIntake();
  chassis.pid_wait();

  chassis.pid_drive_set(-8_in, 80);
  chassis.pid_wait();
  robotactions::setMatchLoaderUp(true);
  pros::delay(100);
  chassis.pid_turn_set(180_deg, 80);
  chassis.pid_wait();
  robotactions::spinIntake(-3000);
  robotactions::setIntakeLiftUp(true);
  pros::delay(200);
  robotactions::setIntakeLiftUp(false);
  robotactions::spinIntake(10000);

  chassis.pid_turn_set(270_deg, 80);
  chassis.pid_wait();
  pros::delay(300);
  robotactions::setMatchLoaderUp(false);

  chassis.pid_drive_set(-20_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-10_in, 60);
  chassis.pid_wait();
  robotactions::setHoodUp(true);
  pros::delay(500);
  robotactions::spinLever(12000);
  pros::delay(700);
  robotactions::spinLever(-robotactions::kFullPowerMv / 2);
  pros::delay(300);
  robotactions::stopLever();

  chassis.pid_drive_set(20_in, DRIVE_SPEED);
  robotactions::setHoodUp(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(17_in, 75);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-3_in, 75);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5_in, 75);
  pros::delay(2500);

  chassis.pid_drive_set(-12_in, 40);
  chassis.pid_wait();
  robotactions::setMatchLoaderUp(true);

  chassis.pid_turn_set(45_deg, 80);
  chassis.pid_wait();
  chassis.pid_drive_set(40_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(26_in, 65);
  chassis.pid_wait();
  robotactions::spinIntake(-3000);
  robotactions::setBasketUp(false);
  pros::delay(300);
  robotactions::setBasketUp(true);
  pros::delay(4000);
  chassis.pid_drive_set(-5_in, 40);
  chassis.pid_wait();
  chassis.pid_drive_set(5_in, 40);

  robotactions::stopIntake();
  robotactions::setIntakeLiftUp(false);
}

void autonRight3() {
  chassis.pid_turn_behavior_set(ez::shortest);
  robotactions::setMatchLoaderUp(true);
  robotactions::spinLever(-robotactions::kFullPowerMv / 2);
  chassis.drive_angle_set(125_deg);

  chassis.pid_drive_set(42_in, DRIVE_SPEED);
  chassis.pid_wait_until(10_in);
  robotactions::spinIntake(robotactions::kFullPowerMv);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(17_in, 40);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, 100);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, 127, 15);
  robotactions::setBasketUp(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30_in, 50);
  chassis.pid_wait();
  robotactions::spinIntake(-4000);
  pros::delay(2300);
  robotactions::stopIntake();

  robotactions::setMatchLoaderUp(false);
  chassis.pid_drive_set(-60_in, DRIVE_SPEED);
  chassis.pid_wait();
  robotactions::spinIntake(9500);
  pros::delay(300);
  chassis.pid_turn_set(270_deg, 80);
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  robotactions::setHoodUp(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12_in, 80);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_relative_set(2, 100);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_relative_set(-2, 100);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(270_deg, 80);
  pros::delay(2000);
  robotactions::stopIntake();
  chassis.pid_wait();

  chassis.pid_drive_set(-8_in, 80);
  chassis.pid_wait();
  robotactions::setMatchLoaderUp(true);
  pros::delay(100);
  chassis.pid_turn_set(180_deg, 80);
  chassis.pid_wait();
  robotactions::setIntakeLiftUp(true);
  robotactions::spinIntake(-4000);
  pros::delay(350);
  robotactions::setIntakeLiftUp(false);
  robotactions::spinIntake(9500);

  chassis.pid_turn_set(270_deg, 80);
  chassis.pid_wait();
  robotactions::setMatchLoaderUp(false);

  chassis.pid_drive_set(-20_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-10_in, 60);
  chassis.pid_wait();
  robotactions::setHoodUp(true);
  pros::delay(500);
  robotactions::spinLever(12000);
  pros::delay(500);
  robotactions::spinLever(-robotactions::kFullPowerMv / 2);
  pros::delay(300);
  robotactions::spinLever(12000);
  pros::delay(500);
  robotactions::spinLever(-robotactions::kFullPowerMv / 2);
  pros::delay(300);
  robotactions::stopLever();

  chassis.pid_drive_set(20_in, DRIVE_SPEED);
  robotactions::setHoodUp(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(18_in, 75);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_relative_set(2, 100);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_relative_set(-2, 100);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(270_deg, 80);
  pros::delay(2500);
  chassis.pid_wait();

  chassis.pid_drive_set(-20_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-14_in, 65);
  chassis.pid_wait();
  robotactions::setHoodUp(true);
  pros::delay(500);
  robotactions::spinLever(12000);
  pros::delay(500);
  robotactions::spinLever(-robotactions::kFullPowerMv / 2);
  pros::delay(500);
  robotactions::spinLever(12000);
  pros::delay(500);
  robotactions::spinLever(-robotactions::kFullPowerMv / 2);
  pros::delay(500);
  robotactions::spinLever(12000);
  pros::delay(500);
  robotactions::spinLever(-robotactions::kFullPowerMv / 2);
  pros::delay(500);
  robotactions::spinLever(12000);
  pros::delay(500);
  robotactions::spinLever(-robotactions::kFullPowerMv / 2);
  pros::delay(500);
  robotactions::stopLever();
  robotactions::stopIntake();
}
