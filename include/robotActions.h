#ifndef ROBOT_ACTIONS_H_
#define ROBOT_ACTIONS_H_

#include "main.h"

#include <cstdint>

namespace robotactions {
constexpr int kFullPowerMv = 12000;
constexpr int kPartialOuttakeMv = 3600;
constexpr int kLowGoalPercent = 70;
constexpr int kLowGoalMv = (kFullPowerMv * kLowGoalPercent) / 100;


void setIntakeVoltages(int voltageMv);
void stopAllIntake();
void spinAllIntake(int voltageMv);
void spinFirstThreeIntake(int voltageMv);

void setLeverVoltages(int voltageMv);
void stopLever();
void spinLever(int voltageMv);

void runIntakeIn();
void runOuttake();
void runLowGoalScore();
void runLowGoalScoreFor(std::uint32_t durationMs);

bool isAnyIntakeRunning();
bool isLeverRunning();
bool isLeverArmUp();

void setMatchLoaderDown(bool down);
void setIntakeLiftUp(bool up);
void setWingFourBarUp(bool up);
void setWingUp(bool up);
void setMidGoalUp(bool up);
void setHoodUp(bool up);

void setLeverUp(bool up);
void setBasketUp(bool up);


} // namespace robotactions

#endif
