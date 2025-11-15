#include "speed_control.hpp"

/**
 * @brief Gets the current robot state
 *
 * @return State enum of the robot
 */
state get_current_state() {
  return current_state;
}

/**
 * @brief Sets a new operational state and corresponding speed
 *
 * @param new_state: The new state to transition to
 * @param new_speed: The speed value for the new state
 */
void set_current_state(state new_state, int new_speed) {
  current_state = new_state;
  current_speed = new_speed;
}