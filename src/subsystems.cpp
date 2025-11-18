#pragma once

#include "subsystems.hpp"

#include "EZ-Template/api.hpp"
#include "api.h"

// Controller mapping functions

// Control only the two bottom intake motors
void control_intake() {
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    // R1 - intake forward
    intake.set_speed(127);  // Forward at full speed
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    // R2 - intake reverse
    intake.set_speed(-127);  // Reverse at full speed
  } else {
    intake.set_speed(0);  // Stop intake
  }
}

// Control all intake motors
void control_full_intake() {
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    // L1 - full intake set_speed (all motors set_speed + hood open)
    intake.set_speed(127);
    conveyor.set_speed(127);
    outtake.set_speed(127);
    hood.set(true);
  } else {
    intake.stop();
    outtake.stop();
    conveyor.stop();
    hood.set(false);
  }
}

void control_intake_pistons() {
  // L2 - extend when held, retract when released
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    intake1.set(true);  // Extend pistons
    intake2.set(true);
  } else {
    intake1.set(false);  // Retract pistons
    intake2.set(false);
  }
}

void control_park_piston() {
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
    // X - toggle park piston
    park.set(!park.get());
  }
}

// Main control function to be called in opcontrol
void update_controls() {
  control_intake();
  control_full_intake();
  control_intake_pistons();
  control_park_piston();
}