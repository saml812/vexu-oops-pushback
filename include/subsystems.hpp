#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "intake.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline Intake intake(1,2);
inline Intake outtake(3,4);
inline Intake conveyor(5,6);

inline ez::Piston lift1('A');
inline ez::Piston lift2('B');

inline ez::Piston intake1('C');
inline ez::Piston intake2('D');

inline ez::Piston hood('E');
inline ez::Piston park('F');

// Controller mapping functions
void control_intake();

void control_full_intake();

void control_intake_pistons();

void control_park_piston();

// Main control function to be called in opcontrol
void main_controls();