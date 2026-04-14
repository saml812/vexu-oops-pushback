#ifndef CONFIGURE_H_
#define CONFIGURE_H_

#include "main.h"
#include "lemlib/api.hpp"

extern pros::Controller master;

extern pros::v5::MotorGroup leftDrive;
extern pros::v5::MotorGroup rightDrive;

extern pros::v5::Motor leftIntake;
extern pros::v5::Motor rightIntake;
// extern pros::v5::Motor leftLever;
// extern pros::v5::Motor rightLever;

// extern pros::v5::MotorGroup intake;
extern pros::v5::MotorGroup lever;

extern pros::IMU inertial;

extern pros::adi::DigitalOut matchLoader;
extern pros::adi::DigitalOut intakeLift;
extern pros::adi::DigitalOut wingFourBar;
extern pros::adi::DigitalOut wing;
extern pros::adi::DigitalOut midGoal;
extern pros::adi::DigitalOut hood;
extern pros::adi::DigitalOut basket;

extern lemlib::Drivetrain drivetrain;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings turnController;

extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;

extern lemlib::Chassis chassis;

void initializeRobot();

#endif
