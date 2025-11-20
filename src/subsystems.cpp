#pragma once

#include "subsystems.hpp"

#include "EZ-Template/api.hpp"
#include "api.h"

// Controller mapping functions

// Control only the two front intake motors
void control_intake() {
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    // R1 - intake forward
    front_intake.set_speed(current_speed);  // Forward at current state speed
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    // R2 - intake reverse
    front_intake.set_speed(-current_speed);  // Reverse at current state speed
  } else {
    front_intake.set_speed(0);  // Stop intake
  }
}

// Control all intake motors
void control_full_intake() {
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    // L1 - full intake set_speed (all motors set_speed + hood open)
    front_intake.set_speed(current_speed);
    bottom_intake.set_speed(current_speed);
    top_intake.set_speed(current_speed);
    hood.set(true);
  } else {
    front_intake.stop();
    bottom_intake.stop();
    top_intake.stop();
    hood.set(false);
  }
}

void control_intake_pistons() {
  // L2 - extend when held, retract when released
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    left_intake_piston.set(true);  // Extend pistons
    right_intake_piston.set(true);
  } else {
    left_intake_piston.set(false);  // Retract pistons
    right_intake_piston.set(false);
  }
}

void control_lift_pistons() {
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    // A - toggle lift pistons
    left_lift_piston.set(!left_lift_piston.get());
    right_lift_piston.set(!right_lift_piston.get());
  }
}

void control_park_piston() {
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
    // X - toggle park piston
    park.set(!park.get());
  }
}

void control_speed() {
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    set_current_state(SCORE_FAST, SCORE_FAST_SPEED);
  } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    set_current_state(SCORE_SLOW, SCORE_SLOW_SPEED);
  }
}

// Main control function to be called in opcontrol
void update_controls() {
  control_intake();
  control_full_intake();
  control_intake_pistons();
  control_lift_pistons();
  control_park_piston();
  control_speed();
}