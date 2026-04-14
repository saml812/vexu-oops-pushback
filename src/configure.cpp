#include "configure.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::v5::MotorGroup leftDrive({-16, 17, -18, 19, -20},
                               pros::v5::MotorGears::blue,
                               pros::v5::MotorUnits::deg);
pros::v5::MotorGroup rightDrive({11, -12, 13, -14, 15},
                                pros::v5::MotorGears::blue,
                                pros::v5::MotorUnits::deg);
// 11-15

pros::v5::Motor leftIntake(10, pros::v5::MotorGears::blue,
                           pros::v5::MotorUnits::deg);
pros::v5::Motor rightIntake(-2, pros::v5::MotorGears::blue,
                            pros::v5::MotorUnits::deg);
// pros::v5::Motor leftLever(9, pros::v5::MotorGears::green,
// pros::v5::MotorUnits::deg); pros::v5::Motor rightLever(-1,
// pros::v5::MotorGears::green, pros::v5::MotorUnits::deg);

// pros::v5::MotorGroup intake({10, -2}, pros::v5::MotorGears::blue,
// pros::v5::MotorUnits::deg);
pros::v5::MotorGroup lever({9, -1}, pros::v5::MotorGears::green,
                           pros::v5::MotorUnits::deg);

pros::IMU inertial(8);

pros::adi::DigitalOut matchLoader('B');
pros::adi::DigitalOut intakeLift('A');
pros::adi::DigitalOut wingFourBar('C');
pros::adi::DigitalOut wing('D');
pros::adi::DigitalOut midGoal('E');
pros::adi::DigitalOut hood('F');
pros::adi::DigitalOut basket('F');
pros::adi::DigitalOut leverPiston('F');

lemlib::Drivetrain drivetrain{&leftDrive, &rightDrive, 12.5, 3, 600, 2};

lemlib::OdomSensors sensors{nullptr, nullptr, nullptr, nullptr, &inertial};

lemlib::ControllerSettings linearController(10, 0, 3, 3, 0.25, 100, 1.5, 500,
                                            0);
// 0,
// 0,
// 0,
// 0,
// 0,
// 0,
// 0,
// 0,
// 0);

lemlib::ControllerSettings turnController(5, 0, 45, 3, 1, 100, 3, 500, 20);
// 0,
// 0,
// 0,
// 0,
// 0,
// 0,
// 0,
// 0,
// 0);

lemlib::ExpoDriveCurve throttleCurve(6, 10, 1.019);

lemlib::ExpoDriveCurve steerCurve(0, 5, 1.019);

lemlib::Chassis chassis(drivetrain, linearController, turnController, sensors,
                        &throttleCurve, &steerCurve);

void initializeRobot() {
    leftDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    rightDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    leftIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // leftLever.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // rightLever.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    lever.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // leftLever.tare_position();
    // rightLever.tare_position();
    lever.tare_position();

    matchLoader.set_value(false);
    intakeLift.set_value(false);
    // wingFourBar.set_value(true);

    wing.set_value(true);
    midGoal.set_value(true);
    hood.set_value(false);
    basket.set_value(false);
    leverPiston.set_value(false);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.calibrate();
    chassis.setPose(0, 0, 0);
}
