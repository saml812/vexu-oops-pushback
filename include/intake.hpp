#pragma

#include "EZ-Template/api.hpp"
#include "api.h"

// Define the motors here
inline pros::Motor left_intake(11); // Define a motor intake on port 11
inline pros::Motor right_intake(12); // Define a motor intake on port 12

inline void set_intake_speed(pros::Motor& intake, int input);

inline void stop_intake(pros::Motor& intake);

void intake_opcontrol();