#pragma once

enum state {
  STOP = 0,
  INTAKE = 1,
  OUTTAKE = 2,
  SCORE = 3,
  SCORE_SLOWLY = 4,
};

inline state current_state = STOP;
inline int current_speed = 0;

state get_current_state();

void set_current_state(state new_state, int new_speed);