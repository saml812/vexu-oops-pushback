#pragma once

/**
 * @brief Speed control for robot
 * 
 * Provides different speed profiles
 */

/// @brief Enumeration of robot speed states
enum state {
  SCORE_FAST = 1,
  SCORE_SLOW = 2,
};

// Speed constants for each state
const int SCORE_FAST_SPEED = 127;
const int SCORE_SLOW_SPEED = 64;

// Global state variables
inline state current_state = SCORE_FAST;  /// Current state
inline int current_speed = 127;           /// Current speed

/**
 * @brief Get the current operational state
 * 
 * @return state Current state enum value
 */
state get_current_state();

/**
 * @brief Set a new state and speed
 * 
 * @param new_state The new state
 * @param new_speed The new speed value
 */
void set_current_state(state new_state, int new_speed);