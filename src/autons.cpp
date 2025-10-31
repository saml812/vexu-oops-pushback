#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int SLOW_INTAKE = 40;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
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
  chassis.slew_drive_constants_set(3_in, 70);
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
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
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
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
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
void match_loader();

// Bottom Bot
void skills_bottom_bot() {
  // Set starting position bot at (-53,-14), orientation: 165 degrees
  chassis.odom_xyt_set(-53_in, -14_in, 165_deg);

  // Move to point (-39.058, -59.5)
  chassis.pid_odom_set({{-39.058_in, 59.606_in}, fwd, DRIVE_SPEED});

  // Turn to point (24.328, -59.5)
  chassis.pid_turn_set({24.328_in, -59.5_in}, fwd, 90);

  // Move to point (24.328, -59.5)
  chassis.pid_odom_set({{24.328_in, -59.5_in}, fwd, DRIVE_SPEED});

  // Turn to point (7.35, -50.362)
  chassis.pid_turn_set({7.35_in, -50.362_in}, fwd, 90);

  // Move to points (24.328, -59.5), (12.443, -53.003), (7.35, -50.363)
  // After passing (12.443, -53.003) --> the intake spins
  // Intakes two blue blocks
  chassis.pid_odom_set({{{24.328_in, -59.5_in}, fwd, DRIVE_SPEED},
                        {{12.443_in, -53.003_in}, fwd, DRIVE_SPEED},
                        {{7.35_in, -50.363_in}, fwd, SLOW_INTAKE}},
                       true);
  chassis.pid_wait_until_index(1);
  // --> Enable intake here
  chassis.pid_wait();

  // Move to point (32.818, -64.511)
  chassis.pid_odom_set({{32.818_in, -64.511_in}, rev, DRIVE_SPEED});

  // Turn to point (44.891, -47.155)
  chassis.pid_turn_set({44.891_in, -47.155_in}, fwd, 90);

  // Move to point (44.891, -47.155)
  chassis.pid_odom_set({{44.891_in, -47.155_in}, fwd, DRIVE_SPEED});

  // Turn to point (60.926, -47)
  chassis.pid_turn_set({60.926_in, -47_in}, fwd, 90);

  // Move to point (60.926, -47)
  chassis.pid_odom_set({{60.926_in, -47_in}, fwd, DRIVE_SPEED});

  // Match Loader / Intake

  // Move to point (44.891, -47.155)
  chassis.pid_odom_set({{44.891_in, -47.155_in}, rev, DRIVE_SPEED});

  // Turn to point (26.592, -47)
  chassis.pid_turn_set({26.592_in, -47_in}, fwd, 90);

  // Move to point (26.592, -47)
  chassis.pid_odom_set({{26.592_in, -47_in}, fwd, DRIVE_SPEED});

  // Score Bottom Long Goal
  // Outtake

  // Move to point (44.891, -47.155)
  chassis.pid_odom_set({{47.155_in, -47_in}, rev, DRIVE_SPEED});

  // Turn to point (47.155, -63.756)
  chassis.pid_turn_set({47.155_in, -63.756_in}, fwd, 90);

  // Move to points (7.155, -47), (47.155, -58.851), (47.155, -63.756)
  // After passing (47.155, -58.851) --> the intake spins
  // Intakes two red blocks
  chassis.pid_odom_set({{{47.155_in, -47_in}, fwd, DRIVE_SPEED},
                        {{47.155_in, -58.851_in}, fwd, DRIVE_SPEED},
                        {{47.155_in, -63.756_in}, fwd, SLOW_INTAKE}},
                       true);
  chassis.pid_wait_until_index(1);
  // Intake on
  chassis.pid_wait();
  // Intake off

  // Move to point (44.891, -47.344)
  chassis.pid_odom_set({{47.155_in, -47.344_in}, rev, DRIVE_SPEED});

  // Turn to point (55.267, -31.308)
  chassis.pid_turn_set({55.267_in, -31.308_in}, fwd, 90);

  // Move to point (55.267, -31.308)
  chassis.pid_odom_set({{55.267_in, -31.308_in}, fwd, DRIVE_SPEED});

  // Move to point (55.267, -12.066)
  chassis.pid_odom_set({{{55.267_in, -12.066_in}, fwd, DRIVE_SPEED},
                        {{55.456_in, 12.081_in}, fwd, SLOW_INTAKE}},
                       true);
  chassis.pid_wait_until(0);
  // Intake on --> 4 red blocks
  chassis.pid_wait();
  // Intake off

  // Move to point (35.27, 34.342)
  chassis.pid_odom_set({{35.27_in, 34.342_in}, fwd, DRIVE_SPEED});

  // Turn to point (9.425, 9.251)
  chassis.pid_turn_set({9.425_in, 9.251_in}, fwd, 90);

  // Move to point (9.425, 9.251)
  chassis.pid_odom_set({{9.425_in, 9.251_in}, fwd, DRIVE_SPEED});

  // Score Middle Goal / Outtake on

  // Move to point (47.155, 47.547)
  chassis.pid_odom_set({{47.155_in, 47.547_in}, rev, DRIVE_SPEED});

  // Turn to point (47.155, 64.714)
  chassis.pid_turn_set({47.155_in, 64.714_in}, fwd, 90);

  // Move to point (47.155, 64.714) with intake on
  chassis.pid_odom_set({{{47.155_in, 47.547_in}, fwd, DRIVE_SPEED},
                        {{47.155_in, 57.54_in}, fwd, DRIVE_SPEED},
                        {{47.155_in, 64.714_in}, fwd, SLOW_INTAKE}},
                       true);
  chassis.pid_wait_until(0);
  // Intake on --> 2 red blocks
  chassis.pid_wait();
  // Intake off

  // Move to point (47.155, 47.547)
  chassis.pid_odom_set({{47.155_in, 47.547_in}, rev, DRIVE_SPEED});
  // Turn to point (60.172, 46.793)
  chassis.pid_turn_set({60.172_in, 46.793_in}, fwd, 90);

  // Move to point (47.155, 47.547)
  chassis.pid_odom_set({{60.172_in, 46.793_in}, fwd, DRIVE_SPEED});

  // Intake

  // Move to point (47.155, 47.547)
  chassis.pid_odom_set({{47.155_in, 47.547_in}, rev, DRIVE_SPEED});
  // Turn to point (27.158, 47.17)
  chassis.pid_turn_set({27.158_in, 47.17_in}, fwd, 90);
  // Move to point (47.155, 47.547)
  chassis.pid_odom_set({{27.158_in, 47.17_in}, fwd, DRIVE_SPEED});

  // Score Top Long Goal
  // Outtake

  // Move to point (47.155, 47.547)
  chassis.pid_odom_set({{47.155_in, 47.547_in}, rev, DRIVE_SPEED});

  // Move to point (-63.205, 20.004) facing 270 degrees
  chassis.pid_odom_set({{-63.205_in, 20.004_in, 45_deg}, fwd, 270});
  chassis.pid_wait();

  // Turn to point (-63.205, 6.799)
  chassis.pid_turn_set({-63.205_in, 6.799_in}, fwd, 90);
  // Move to point (-63.205, 6.799)
  chassis.pid_odom_set({{-63.205_in, 6.799_in}, fwd, DRIVE_SPEED});
  // Parked
}

// Top Bot
void skills_top_bot() {
  // Set starting position bot at (-54,11.892), orientation: 20 degrees
  chassis.odom_xyt_set(-54_in, 11.892_in, 20_deg);

  // Move to point (-44.529, 46.981)
  chassis.pid_odom_set({{-44.529_in, 46.981_in}, fwd, DRIVE_SPEED});

  // Turn to point (-62.639, 46.793)
  chassis.pid_turn_set({-62.639_in, 46.79_in}, fwd, 90);

  // Move to point (-62.639, 46.79)
  chassis.pid_odom_set({{-62.639_in, 46.79_in}, fwd, DRIVE_SPEED});

  // Intake On

  // Move to point (-47.17, 46.981)
  chassis.pid_odom_set({{-47.17_in, 46.981_in}, rev, DRIVE_SPEED});

  // Turn to point (-27.55, 46.793)
  chassis.pid_turn_set({-27.55_in, 46.79_in}, fwd, 90);

  // Move to point (-47.17, 46.981)
  chassis.pid_odom_set({{-27.55_in, 46.79_in}, fwd, DRIVE_SPEED});

  // Score Top Long Goal
  // Outtake

  // Move to point (-47.17, 46.981)
  chassis.pid_odom_set({{-47.17_in, 46.981_in}, rev, DRIVE_SPEED});
  // Turn to point (-47.17, 63.205)
  chassis.pid_turn_set({-47.17_in, 63.205_in}, fwd, 90);

  // Move to point (47.155, 64.714) with intake on
  chassis.pid_odom_set({{{-47.17_in, 46.981_in}, fwd, DRIVE_SPEED},
                        {{-47.17_in, 59.352_in}, fwd, DRIVE_SPEED},
                        {{-47.17_in, 63.205_in}, fwd, SLOW_INTAKE}},
                       true);
  chassis.pid_wait_until(0);
  // Intake on --> 2 blue blocks
  chassis.pid_wait();
  // Intake off

  // Move to point (-56.225, 30.946)
  chassis.pid_odom_set({{-56.225_in, 30.946_in}, rev, DRIVE_SPEED});

  // Turn to point (-56.037, 11.327)
  chassis.pid_turn_set({-56.037_in, 11.327_in}, fwd, 90);

  // Move to point (-56.414, -13.198) with intake on
  chassis.pid_odom_set({{{-56.225_in, 30.946_in}, fwd, DRIVE_SPEED},
                        {{-56.037_in, 11.327_in}, fwd, DRIVE_SPEED},
                        {{-56.414_in, -13.198_in}, fwd, SLOW_INTAKE}},
                       true);
  chassis.pid_wait_until(0);
  // Intake on --> 4 blue blocks
  chassis.pid_wait();
  // Intake off

  // Turn to point (-26.796, 28.682)
  chassis.pid_turn_set({-26.796_in, 28.682_in}, fwd, 90);

  // Move to point (-26.796, 28.682)
  chassis.pid_odom_set({{-26.796_in, 28.682_in}, fwd, DRIVE_SPEED});

  // Turn to point (-10.383, 10.383)
  chassis.pid_turn_set({-10.383_in, 10.383_in}, fwd, 90);

  // Move to point (-10.383, 10.383)
  chassis.pid_odom_set({{-10.383_in, 10.383_in}, fwd, DRIVE_SPEED});

  // Score Middle Goal
  // Outtake

  // Move to point (-33.399, 34.342)
  chassis.pid_odom_set({{-33.399_in, 34.342_in}, rev, DRIVE_SPEED});

  // Turn to point (-46.604, -47.532)
  chassis.pid_turn_set({-46.604_in, -47.532_in}, fwd, 90);

  // Move to point (-47.17, -63.002)
  chassis.pid_odom_set({{-47.17_in, -63.002_in, 180_deg}, fwd, DRIVE_SPEED});

  // Intake two blue blocks

  // Move to point (-46.604, -47.532)
  chassis.pid_odom_set({{-46.604_in, -47.532_in}, rev, DRIVE_SPEED});

  // Turn to point (-62.262, -47.532)
  chassis.pid_turn_set({-62.262_in, -47.532_in}, fwd, 90);

  // Move to point (-62.262, -47.532)
  chassis.pid_odom_set({{-46.604_in, -47.532_in}, fwd, DRIVE_SPEED});

  // Intake Match Loader

  // Move to point (-46.604, -47.532)
  chassis.pid_odom_set({{-46.604_in, -47.532_in}, rev, DRIVE_SPEED});
  // Turn to point (-27.55, -47.155)
  chassis.pid_turn_set({-27.55_in, -47.155_in}, fwd, 90);

  // Move to point (-27.55, -47.155)
  chassis.pid_odom_set({{-27.55_in, -47.155_in}, fwd, DRIVE_SPEED});

  // Score Bottom Long Goal
  // Outtake

  // Move to point (-63.205, -27.347)
  chassis.pid_odom_set({{-63.205_in, -27.347_in}, rev, DRIVE_SPEED});
  // Turn to point (-63.205, -6.029)
  chassis.pid_turn_set({-63.205_in, -6.029_in}, fwd, 90);
  // Move to point (-63.205, 0.196)
  chassis.pid_odom_set({{-63.205_in, 0.196_in}, fwd, DRIVE_SPEED});
  // Parked
}

// Bottom Bot
void head_two_head_bottom(const std::string& color) {
  // Mirror Auto
  if (color == "blue") {
    chassis.odom_x_flip();
    chassis.odom_theta_flip();
  }
  // Set starting position bot at (-51, -10), orientation: 110 degrees
  chassis.odom_xyt_set(-51_in, -10_in, 110_deg);

  // Move to point (-32.078, -16.216)
  chassis.pid_odom_set({{-32.078_in, -16.216_in}, fwd, DRIVE_SPEED});

  // Move to point (-0.008, -37.722) with intake on
  chassis.pid_odom_set({{{-8.422_in, -19.961_in}, fwd, DRIVE_SPEED},
                        {{-1.78_in, -26.201_in}, fwd, SLOW_INTAKE},
                        {{-0.008_in, -37.722_in}, fwd, SLOW_INTAKE}},
                       true);
  chassis.pid_wait_until(0);
  // Intake on --> 3 red blocks
  chassis.pid_wait();
  // Intake off

  // Move to point (0.032, -31.838)
  chassis.pid_odom_set({{0.032_in, -31.838_in}, rev, DRIVE_SPEED});
  // Turn to point (-27.362, -27.724)
  chassis.pid_turn_set({-27.362_in, -27.724_in}, fwd, 90);
  // Move to point (-27.362, -27.724)
  chassis.pid_odom_set({{-27.362_in, -27.724_in}, rev, DRIVE_SPEED});
  // Turn to point (-10.761, -11.123)
  chassis.pid_turn_set({-10.761_in, -11.123_in}, fwd, 90);
  // Move to point (-10.761, -11.123)
  chassis.pid_odom_set({{-10.761_in, -11.123_in}, fwd, DRIVE_SPEED});
  // Score Middle Goal
  // Outtake

  // Move to point (-47.736, -46.966)
  chassis.pid_odom_set({{-47.736_in, -46.966_in}, rev, DRIVE_SPEED});
  // Turn to point (-61.319, -46.966)
  chassis.pid_turn_set({-61.319_in, -46.966_in}, fwd, 90);

  // Move to point (-61.319, -46.966)
  chassis.pid_odom_set({{-61.319_in, -46.966_in}, fwd, DRIVE_SPEED});
  // Match Loader Intake

  // Move to point (-47.736, -46.966)
  chassis.pid_odom_set({{-47.736_in, -46.966_in}, rev, DRIVE_SPEED});
  // Turn to point (-27.173, -47.344)
  chassis.pid_turn_set({-27.173_in, -47.344_in}, fwd, 90);
  // Move to point (-27.173, -47.344)
  chassis.pid_odom_set({{-27.173_in, -47.344_in}, fwd, DRIVE_SPEED});
  // Score Bottom Long Goal
  // Outtake

  // Move to point (-52.83, -36.779)
  chassis.pid_odom_set({{-52.83_in, -36.779_in}, rev, DRIVE_SPEED});
  // Turn to point (-62.639, -21.687)
  chassis.pid_turn_set({-62.639_in, -21.687_in}, fwd, 90);

  // Move to point (-62.828, 0.196)
  chassis.pid_odom_set({{-62.828_in, 0.196_in, 0_deg}, fwd, DRIVE_SPEED});
  // Finish
}

// Top Bot
void head_two_head_top(const std::string& color) {
  if (color == "blue") {
    chassis.odom_x_flip();
    chassis.odom_theta_flip();
  }

  // Set starting position bot at (-52.452, 11.138), orientation: 45 degrees
  chassis.odom_xyt_set(-52.452_in, 11.138_in, 45_deg);

  // Move to point (-5.667, 21.136)
  chassis.pid_odom_set({{-5.667_in, 21.136_in, 45_deg}, fwd, DRIVE_SPEED});

  // Move to point (-0.196, 36.794) with intake on
  chassis.pid_odom_set({{{-5.667_in, 21.136_in}, fwd, DRIVE_SPEED},
                        {{-0.169_in, -29.157_in}, fwd, SLOW_INTAKE},
                        {{-0.196_in, 36.794_in}, fwd, SLOW_INTAKE}},
                       true);
  chassis.pid_wait_until(0);
  // Intake on --> 3 red blocks
  chassis.pid_wait();
  // Intake off

  // Turn to point (-34.342, 36.228)
  chassis.pid_turn_set({-34.342_in, 36.228_in}, fwd, 90);

  // Move to point (-34.342, 36.228)
  chassis.pid_odom_set({{-34.342_in, 36.228_in}, fwd, DRIVE_SPEED});

  // Move to point (-45.472, 46.981)
  chassis.pid_odom_set({{-45.472_in, 46.981_in}, fwd, DRIVE_SPEED});

  // Turn to point (-62.828, 46.981)
  chassis.pid_turn_set({-62.828_in, 46.981_in}, fwd, 90);

  // Move to point (-62.828, 46.981)
  chassis.pid_odom_set({{-62.828_in, 46.981_in}, fwd, DRIVE_SPEED});

  // Match Loader Intake

  // Move to point (-45.472, 46.981)
  chassis.pid_odom_set({{-45.472_in, 46.981_in}, rev, DRIVE_SPEED});

  // Turn to point (-25.098, 47.359)
  chassis.pid_turn_set({-25.098_in, 47.359_in}, fwd, 90);
  // Move to point(-25.098, 47.359)
  chassis.pid_odom_set({{-25.098_in, 47.359_in}, fwd, DRIVE_SPEED});

  // Score Top Long Goal
  // Outtake

  // Finish
}
