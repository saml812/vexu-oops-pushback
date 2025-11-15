/**
 * @file intake.cpp
 * @brief Implementation of the Intake class for robot intake mechanism
 */

#include "intake.hpp"

/**
 * @brief Construct a new Intake object
 * 
 * Initializes both intake motors with specified ports and configures their brake modes
 * 
 * @param left_port Port number for left intake motor
 * @param right_port Port number for right intake motor
 */
Intake::Intake(int left_port, int right_port) 
    : left_motor(left_port), right_motor(right_port) {
    // Set motor brake modes to coast when not powered
    left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

/**
 * @brief Sets speed for both intake motors
 * 
 * @param speed Motor speed value (-127 to 127)
 */
void Intake::set_speed(int speed) {
    left_motor.move(speed);
    right_motor.move(speed);
}

/**
 * @brief Stops both intake motors
 * 
 * Sets both motor speeds to zero
 */
void Intake::stop() {
    left_motor.move(0);
    right_motor.move(0);
}