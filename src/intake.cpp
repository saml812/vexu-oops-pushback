#include "main.h"

void set_intake_speed(pros::Motor& intake, int input) {
  intake.move(input);
}

void stop_intake(pros::Motor& intake) {
  intake.move(0);
}

// void intake_opcontrol() {
//   if (master.get_digital(DIGITAL_L1)) {
//     set_intake_speed(127);
//   } else if (master.get_digital(DIGITAL_L2)) {
//     set_intake_speed(-127);
//   } else {
//     set_intake_speed(0);
//   }
// }