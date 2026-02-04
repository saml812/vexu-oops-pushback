#pragma once

#include "subsystems.hpp"

#include "EZ-Template/api.hpp"
#include "api.h"

// Controller mapping functions

// Control all motors and intake pistons
void control_intakes() {
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    // R1 - intake forward
    left_front_intake.move(current_speed);  // Forward at current state speed
    right_front_intake.move(current_speed);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    // R2 - intake reverse
    left_front_intake.move(-current_speed);  // Reverse at current state speed
    right_front_intake.move(-current_speed);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {  // L1 - full intake spin + hood open
    left_front_intake.move(current_speed);
    right_front_intake.move(current_speed);
    left_bottom_intake.move(current_speed);
    right_bottom_intake.move(current_speed);
    left_top_intake.move(current_speed);
    right_top_intake.move(current_speed);
    hood.set(true);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {  // L2 - extend when held, retract when released
    left_intake_piston.set(true);                                  // Extend pistons
    right_intake_piston.set(true);
  } else {
    left_front_intake.move(0);  // Stop intake
    right_front_intake.move(0);
    left_bottom_intake.move(0);
    right_bottom_intake.move(0);
    left_top_intake.move(0);
    right_top_intake.move(0);

    left_intake_piston.set(false);  // Retract intake pistons
    right_intake_piston.set(false);

    hood.set(false);
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

void control_descore_piston() {
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
    // X - toggle park piston -- ask issac what button to assign
    descore.set(!descore.get());
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
void main_controls() {
  control_intakes();
  control_lift_pistons();
  control_park_piston();
  control_descore_piston();
  control_speed();
}