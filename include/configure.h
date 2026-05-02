#ifndef CONFIGURE_H_
#define CONFIGURE_H_

#include "EZ-Template/api.hpp"
#include "main.h"

extern pros::Controller master;

extern pros::v5::MotorGroup leftDrive;
extern pros::v5::MotorGroup rightDrive;

extern pros::v5::Motor leftIntake;
extern pros::v5::Motor rightIntake;
extern pros::v5::MotorGroup lever;

extern pros::IMU inertial;
extern pros::Distance distanceSensor;

extern pros::adi::DigitalOut matchLoader;
extern pros::adi::DigitalOut intakeLift;
extern pros::adi::DigitalOut wingFourBar;
extern pros::adi::DigitalOut wingL;
extern pros::adi::DigitalOut wingR;
extern pros::adi::DigitalOut hood;
extern pros::adi::DigitalOut basket;
extern pros::adi::DigitalOut leverPiston;

extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;

extern ez::Drive chassis;

void initializeRobot();

#endif
