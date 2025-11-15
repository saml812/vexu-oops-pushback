#pragma once

// Enum states for different modes
enum state {
  STOP = 0,           // System is stopped
  INTAKE = 1,         // Intaking objects
  SCORE = 2,          // Scoring at max speed
  SCORE_SLOWLY = 3,   // Scoring at slow speed
};

// Variables for current state and speed
inline state current_state = STOP;
inline int current_speed = 0;

// Get and Set functions
state get_current_state();
void set_current_state(state new_state, int new_speed);