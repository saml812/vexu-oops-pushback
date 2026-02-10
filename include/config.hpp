#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/misc.h"

// Drivetrain motors
inline const std::vector<int> drive_ports_left = {11, -12, 13, -14, 15};    // Left ports
inline const std::vector<int> drive_ports_right = {-16, 17, -18, 19, -20};  // Right ports

inline pros::MotorGroup left_dt({11, -12, 13, -14, 15});
inline pros::MotorGroup right_dt({-16, 17, -18, 19, -20});

inline const int imu_port = 2;         // IMU Port
inline const double wheel_dia = 4.12;  // Wheel diameter
inline const double wheel_rpm = 600;   // Wheel RPM

inline pros::motor_brake_mode_e_t opcontrol_brake = pros::E_MOTOR_BRAKE_BRAKE;  // Chassis brake most for opcontrol

inline bool curve_buttons_toggle = false;                             // Enables modifying the controller curve with buttons on the joysticks
inline double active_brake_constant = 2.0;                            // Sets the active brake kP. We recommend ~2.  0 will disable
inline std::pair<double, double> drive_curve_constants = {0.0, 0.0};  // Defaults for curve. If using tank, only the first parameter is used

extern Drive chassis;
extern pros::Controller master;

// Intake motors
inline pros::Motor left_bottom_intake(1);
inline pros::Motor right_bottom_intake(2);
inline pros::Motor left_top_intake(3);
inline pros::Motor right_top_intake(4);
inline pros::MotorGroup intake({1, 2, 3, 4});

// Pneumatic pistons
inline ez::Piston left_lift_piston('A');   // Left side lift piston
inline ez::Piston right_lift_piston('B');  // Right side lift piston

// Match loader pistons
inline ez::Piston left_match_loader('C');   // Left intake piston
inline ez::Piston right_match_loader('D');  // Right intake piston

inline ez::Piston hood('E');     // Hood piston
inline ez::Piston park('F');     // Parking piston
inline ez::Piston descore('G');  // Descore piston
