#pragma once

void default_constants();

void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();

// custom autos
void match_loader();
void skills_bottom_bot();
void skills_top_bot();
void head_two_head_bottom(const std::string& color);
void head_two_head_top(const std::string& color);
