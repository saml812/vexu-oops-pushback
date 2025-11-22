#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "intake.hpp"
#include "speed_control.hpp"

extern Drive chassis;  // Drivetrain chassis object

// /**
//  * @brief Subsystem declarations
//  *
//  * Contains all motor controllers, pistons, and subsystem objects and functions for the robot
//  */

// // Intake subsystem motor groups
// // extern Intake front_intake(1, 2);   // Front intake mechanism
// // extern Intake bottom_intake(3, 4);  // Bottom intake mechanism
// // extern Intake top_intake(5, 6);     // Top intake mechanism




// // Pneumatic piston controls
// extern ez::Piston left_lift_piston('A');   // Left side lift piston
// extern ez::Piston right_lift_piston('B');  // Right side lift piston

// extern ez::Piston left_intake_piston('C');   // Left intake piston
// extern ez::Piston right_intake_piston('D');  // Right intake piston

// extern ez::Piston hood('E');  // Hood piston
// extern ez::Piston park('F');  // Parking piston

// // Controller mapping functions

// /**
//  * @brief Control front intakes
//  */
// void control_intake();

// /**
//  * @brief Control all intakes simultaneously
//  */
// void control_full_intake();

// /**
//  * @brief Control intake pistons
//  */
// void control_intake_pistons();

// /**
//  * @brief Control lift pistons
//  */
// void control_lift_pistons();

// /**
//  * @brief Control park piston
//  */
// void control_park_piston();

// /**
//  * @brief Control speed states
//  */
// void control_speed();

// /**
//  * @brief Main control function to be called in opcontrol
//  *
//  * Calls all subsystem controls functions
//  */
// void main_controls();

// // Autonomous control functions
// // void auton_control_intake(const std::string& state);
// // void auton_control_bottom_intake(const std::string& state);
// // void auton_control_top_intake(const std::string& state);
// // void auton_control_all_intakes(const std::string& state);
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