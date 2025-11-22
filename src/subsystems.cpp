// #pragma once

// #include "subsystems.hpp"

// #include "EZ-Template/api.hpp"
// #include "api.h"

// // Controller mapping functions

// // Control only the two front intake motors
// void control_intake() {
//   if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
//     // R1 - intake forward
//     front_intake.set_speed(current_speed);  // Forward at current state speed
//   } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
//     // R2 - intake reverse
//     front_intake.set_speed(-current_speed);  // Reverse at current state speed
//   } else {
//     front_intake.set_speed(0);  // Stop intake
//   }
// }

// // Control all intake motors
// void control_full_intake() {
//   if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
//     // L1 - full intake set_speed (all motors set_speed + hood open)
//     front_intake.set_speed(current_speed);
//     bottom_intake.set_speed(current_speed);
//     top_intake.set_speed(current_speed);
//     hood.set(true);
//   } else {
//     front_intake.stop();
//     bottom_intake.stop();
//     top_intake.stop();
//     hood.set(false);
//   }
// }

// void control_intake_pistons() {
//   // L2 - extend when held, retract when released
//   if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
//     left_intake_piston.set(true);  // Extend pistons
//     right_intake_piston.set(true);
//   } else {
//     left_intake_piston.set(false);  // Retract pistons
//     right_intake_piston.set(false);
//   }
// }

// void control_lift_pistons() {
//   if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
//     // A - toggle lift pistons
//     left_lift_piston.set(!left_lift_piston.get());
//     right_lift_piston.set(!right_lift_piston.get());
//   }
// }

// void control_park_piston() {
//   if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
//     // X - toggle park piston
//     park.set(!park.get());
//   }
// }

// void control_speed() {
//   if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
//     set_current_state(SCORE_FAST, SCORE_FAST_SPEED);
//   } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
//     set_current_state(SCORE_SLOW, SCORE_SLOW_SPEED);
//   }
// }

// // Main control function to be called in opcontrol
// void update_controls() {
//   control_intake();
//   control_full_intake();
//   control_intake_pistons();
//   control_lift_pistons();
//   control_park_piston();
//   control_speed();
// }

// // Autonomous control functions

// // /**
// //  * @brief Control front intake in autonomous
// //  * @param state "on" to activate, "off" to deactivate
// //  */
// // void auton_control_intake(const std::string& state) {
// //   if (state == "forward") {
// //     front_intake.set_speed(127);  // Full forward
// //   } else if (state == "off") {
// //     front_intake.set_speed(0);  // Stop
// //   } else if (state == "reverse") {
// //     front_intake.set_speed(-127);  // Full reverse
// //   }
// // }

// // /**
// //  * @brief Control bottom intake in autonomous
// //  * @param state "on" to activate, "off" to deactivate
// //  */
// // void auton_control_bottom_intake(const std::string& state) {
// //   if (state == "forward") {
// //     bottom_intake.set_speed(127);  // Full forward
// //   } else if (state == "off") {
// //     bottom_intake.set_speed(0);  // Stop
// //   } else if (state == "reverse") {
// //     bottom_intake.set_speed(-127);  // Full reverse
// //   }
// // }

// // /**
// //  * @brief Control top intake in autonomous
// //  * @param state "on" to activate, "off" to deactivate
// //  */
// // void auton_control_top_intake(const std::string& state) {
// //   if (state == "forward") {
// //     top_intake.set_speed(127);  // Full forward
// //   } else if (state == "off") {
// //     top_intake.set_speed(0);  // Stop
// //   } else if (state == "reverse") {
// //     top_intake.set_speed(-127);  // Full reverse
// //   }
// // }

// // /**
// //  * @brief Control all intakes simultaneously in autonomous
// //  * @param state "on" to activate, "off" to deactivate
// //  */
// // void auton_control_all_intakes(const std::string& state) {
// //   auton_control_intake(state);
// //   auton_control_bottom_intake(state);
// //   auton_control_top_intake(state);
// // }

// /**
//  * @brief Control intake with specific speed in autonomous
//  * @param speed Motor speed (-127 to 127)
//  */
// void auton_control_intake_speed(int speed) {
//   front_intake.set_speed(speed);
// }

// /**
//  * @brief Control bottom intake with specific speed in autonomous
//  * @param speed Motor speed (-127 to 127)
//  */
// void auton_control_bottom_intake_speed(int speed) {
//   bottom_intake.set_speed(speed);
// }

// /**
//  * @brief Control top intake with specific speed in autonomous
//  * @param speed Motor speed (-127 to 127)
//  */
// void auton_control_top_intake_speed(int speed) {
//   top_intake.set_speed(speed);
// }

// /**
//  * @brief Control all intakes with specific speed in autonomous
//  * @param speed Motor speed (-127 to 127)
//  */
// void auton_control_all_intakes_speed(int speed) {
//   front_intake.set_speed(speed);
//   bottom_intake.set_speed(speed);
//   top_intake.set_speed(speed);
// }

// /**
//  * @brief Control intake pistons in autonomous
//  * @param state "extend" or "retract"
//  */
// void auton_control_intake_pistons(const std::string& state) {
//   if (state == "extend") {
//     left_intake_piston.set(true);
//     right_intake_piston.set(true);
//   } else if (state == "retract") {
//     left_intake_piston.set(false);
//     right_intake_piston.set(false);
//   }
// }

// /**
//  * @brief Control lift pistons in autonomous
//  * @param state "extend" or "retract"
//  */
// void auton_control_lift_pistons(const std::string& state) {
//   if (state == "extend") {
//     left_lift_piston.set(true);
//     right_lift_piston.set(true);
//   } else if (state == "retract") {
//     left_lift_piston.set(false);
//     right_lift_piston.set(false);
//   }
// }

// /**
//  * @brief Control hood piston in autonomous
//  * @param state "extend" or "retract"
//  */
// void auton_control_hood(const std::string& state) {
//   if (state == "extend") {
//     hood.set(true);
//   } else if (state == "retract") {
//     hood.set(false);
//   }
// }

// /**
//  * @brief Control park piston in autonomous
//  * @param state "extend" or "retract"
//  */
// void auton_control_park(const std::string& state) {
//   if (state == "extend") {
//     park.set(true);
//   } else if (state == "retract") {
//     park.set(false);
//   }
// }

// /**
//  * @brief Set chassis speed state in autonomous
//  * @param state "fast", "slow", or "normal"
//  */
// void auton_control_speed(const std::string& state) {
//   if (state == "fast") {
//     set_current_state(SCORE_FAST, SCORE_FAST_SPEED);
//   } else if (state == "slow") {
//     set_current_state(SCORE_SLOW, SCORE_SLOW_SPEED);
//   }
// }

// /**
//  * @brief Stop all intakes in autonomous
//  */
// void auton_stop_all_intakes() {
//   front_intake.set_speed(0);
//   bottom_intake.set_speed(0);
//   top_intake.set_speed(0);
// }