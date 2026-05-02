#include "configure.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::v5::MotorGroup leftDrive({-20, 19, -18, 17, -16},
                               pros::v5::MotorGears::blue,
                               pros::v5::MotorUnits::deg);
pros::v5::MotorGroup rightDrive({11, -12, 13, -14, 15},
                                pros::v5::MotorGears::blue,
                                pros::v5::MotorUnits::deg);

pros::v5::Motor leftIntake(9, pros::v5::MotorGears::blue,
                           pros::v5::MotorUnits::deg);
pros::v5::Motor rightIntake(-4, pros::v5::MotorGears::blue,
                            pros::v5::MotorUnits::deg);
pros::v5::MotorGroup lever({8, -2}, pros::v5::MotorGears::green,
                           pros::v5::MotorUnits::deg);

pros::IMU inertial(21);

pros::Distance distanceSensor(1);

pros::adi::DigitalOut matchLoader('F');
pros::adi::DigitalOut intakeLift('G');
pros::adi::DigitalOut wingL('C', LOW);  // Left wing
pros::adi::DigitalOut wingR('D', LOW);  // Right wing
pros::adi::DigitalOut hood('A');
pros::adi::DigitalOut basket('H');
pros::adi::DigitalOut leverPiston('B');

ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-20, 19, -18, 17, -16},  // Left Chassis Ports (negative port will reverse it!)
    {11, -12, 13, -14, 15},   // Right Chassis Ports (negative port will reverse it!)

    21,    // IMU Port
    2.75,  // Wheel Diam`eter (Remember, 4" wheels without screw holes are actually 4.125!)
    600);

// 5, 0, 45, 3, 1, 100, 3, 500, 20

// Deadband, minOutput, Curve
lemlib::ExpoDriveCurve throttleCurve(6, 20, 1.019);
lemlib::ExpoDriveCurve steerCurve(0, 20, 1.019);

// // Ivan driveCurve
// lemlib::ExpoDriveCurve throttleCurve2(0, 10, 1.002);
// // Ivan steerCurve
// lemlib::ExpoDriveCurve steerCurve2(0, 10, 1);

void initializeRobot() {
  leftDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
  rightDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

  leftIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rightIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  lever.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

  lever.set_zero_position_all(0.0);
  lever.tare_position_all();

  matchLoader.set_value(true);
  intakeLift.set_value(false);
  // wingFourBar.set_value(true);

  //   wingL.set_value(false);
  //   wingR.set_value(false);

  hood.set_value(false);
  basket.set_value(false);
  //   leverPiston.set_value(false);

  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
}
