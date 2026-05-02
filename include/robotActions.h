#ifndef ROBOT_ACTIONS_H_
#define ROBOT_ACTIONS_H_

#include <cstdint>

#include "pros/adi.hpp"

namespace robotactions {

constexpr int kFullPowerMv = 12000;
constexpr int kPartialOuttakeMv = 3600;
constexpr int kLowGoalMv = (kFullPowerMv * 70) / 100;

void setIntakeVoltage(int voltageMv);
void setLeverVoltage(int voltageMv);
void stopIntake();
void stopLever();
void spinIntake(int voltageMv);
void spinLever(int voltageMv);

void runIntakeIn();
void runOuttake();
void runLowGoalScore();
void runLowGoalScoreFor(uint32_t durationMs);

bool isAnyIntakeRunning();
bool isLeverRunning();

void setMatchLoaderUp(bool down);
void setIntakeLiftUp(bool up);
// void setWingFourBarUp(bool up);
void setWingUp(bool up, pros::adi::DigitalOut wing);
void setHoodUp(bool up);
void setBasketUp(bool up);
// void setLeverPistonUp(bool up);

}  // namespace robotactions

#endif