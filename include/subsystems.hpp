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