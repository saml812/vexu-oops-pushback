#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "speed_control.hpp"
#include "config.hpp"

extern Drive chassis;  // Drivetrain chassis object

/**
 * @brief Subsystem declarations
 *
 * Contains all motor controllers, pistons, and subsystem objects and functions for the robot
 */

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
 * @brief Control intake speed states
 */
void control_speed();

/**
 * @brief Main control function to be called in opcontrol
 *
 * Calls all subsystem controls functions
 */
void main_controls();

void arcadeDrive(double deadband);