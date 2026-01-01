#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "speed_control.hpp"

extern Drive chassis;  // Drivetrain chassis object

/**
 * @brief Subsystem declarations
 *
 * Contains all motor controllers, pistons, and subsystem objects and functions for the robot
 */

// Intake motors
inline pros::Motor left_front_intake(1);    //
inline pros::Motor right_front_intake(2);   //
inline pros::Motor left_bottom_intake(3);   //
inline pros::Motor right_bottom_intake(4);  //
inline pros::Motor left_top_intake(5);      //
inline pros::Motor right_top_intake(6);     //

// Pneumatic pistons
inline ez::Piston left_lift_piston('A');   // Left side lift piston
inline ez::Piston right_lift_piston('B');  // Right side lift piston

inline ez::Piston left_intake_piston('C');   // Left intake piston
inline ez::Piston right_intake_piston('D');  // Right intake piston

inline ez::Piston hood('E');     // Hood piston
inline ez::Piston park('F');     // Parking piston
inline ez::Piston descore('G');  // Descore piston

// Controller mapping functions

/**
 * @brief Control front intakes
 */
void control_intakes();

/**
 * @brief Control lift pistons
 */
void control_lift_pistons();

/**
 * @brief Control park piston
 */
void control_park_piston();

/**
 * @brief Control descore piston
 */
void control_descore_piston();

/**
 * @brief Control speed states
 */
void control_speed();

/**
 * @brief Main control function to be called in opcontrol
 *
 * Calls all subsystem controls functions
 */
void main_controls();

// Autonomous control functions
// void auton_control_intake(const std::string& state);
// void auton_control_bottom_intake(const std::string& state);
// void auton_control_top_intake(const std::string& state);
// void auton_control_all_intakes(const std::string& state);
// void auton_control_intake_speed(const int speed);
// void auton_control_bottom_intake_speed(const int speed);
// void auton_control_top_intake_speed(const int speed);
// void auton_control_all_intakes_speed(const int speed);

// void auton_control_intake_pistons(const std::string& state);
// void auton_control_lift_pistons(const std::string& state);
// void auton_control_hood(const std::string& state);
// void auton_control_park(const std::string& state);
// void auton_control_speed(const std::string& state);
// void auton_stop_all_intakes();