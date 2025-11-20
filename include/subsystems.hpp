#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "intake.hpp"
#include "speed_control.hpp"

extern Drive chassis;  // Drivetrain chassis object

/**
 * @brief Subsystem declarations
 *
 * Contains all motor controllers, pistons, and subsystem objects and functions for the robot
 */

// Intake subsystem motor groups
inline Intake front_intake(1, 2);   // Front intake mechanism
inline Intake bottom_intake(3, 4);  // Bottom intake mechanism
inline Intake top_intake(5, 6);     // Top intake mechanism

// Pneumatic piston controls
inline ez::Piston left_lift_piston('A');   // Left side lift piston
inline ez::Piston right_lift_piston('B');  // Right side lift piston

inline ez::Piston left_intake_piston('C');   // Left intake piston
inline ez::Piston right_intake_piston('D');  // Right intake piston

inline ez::Piston hood('E');  // Hood piston
inline ez::Piston park('F');  // Parking piston

// Controller mapping functions

/**
 * @brief Control front intakes
 */
void control_intake();

/**
 * @brief Control all intakes simultaneously
 */
void control_full_intake();

/**
 * @brief Control intake pistons
 */
void control_intake_pistons();

/**
 * @brief Control lift pistons
 */
void control_lift_pistons();

/**
 * @brief Control park piston
 */
void control_park_piston();

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