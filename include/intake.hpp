/**
 * @file intake.hpp
 * @brief Declaration of the Intake class for robot intake mechanism control
 */

#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "speed_control.hpp"
#include "main.h"

/**
 * @brief Intake motor controller class for managing left and right intake motors
 * 
 * This class provides a interface for controlling the robot's intake mechanism,
 * including forward/reverse operation and individual motor control.
 */
class Intake {
private:
    pros::Motor left_motor;   // Left intake motor object
    pros::Motor right_motor;  // Right intake motor object

    // pros::Motor middle_left_motor;
    // pros::Motor middle_right_motor;

    // pros::Motor top_left_motor;
    // pros::Motor top_right_motr;
    
    // ez::PistonGroup intake;
    // ez::PistonGroup lift;

    // ez::Piston hood;
    // ez::Piston park;

public:
    /**
     * @brief Construct a new Intake object
     * 
     * @param left_port Port number for left intake motor
     * @param right_port Port number for right intake motor
     */
    Intake(int left_port, int right_port);

    /**
     * @brief Sets speed for both intake motors
     * 
     * @param speed Motor speed value (-127 to 127)
     */
    void set_speed(int speed);

    /**
     * @brief Stops both intake motors
     */
    void stop();
    
    /**
     * @brief Gets the left intake motor object
     * 
     * @return Reference to left intake motor
     */
    pros::Motor& get_left_motor() { return left_motor; }

    /**
     * @brief Gets the right intake motor object
     * 
     * @return Reference to right intake motor
     */
    pros::Motor& get_right_motor() { return right_motor; }
};