#include "robotActions.h"

#include <cstdint>

#include "configure.h"

namespace {
bool intakeRunning = false;
bool leverRunning = false;
}  // namespace

namespace robotactions {
void setIntakeVoltage(int voltageMv) {
  leftIntake.move_voltage(voltageMv);
  rightIntake.move_voltage(voltageMv);
  intakeRunning = voltageMv != 0;
}

void setLeverVoltage(int voltageMv) {
  lever.move_voltage(voltageMv);
  leverRunning = voltageMv != 0;
}

void stopIntake() { setIntakeVoltage(0); }

void stopLever() { setLeverVoltage(0); }

void spinIntake(int voltageMv) { setIntakeVoltage(voltageMv); }

void spinLever(int voltageMv) { setLeverVoltage(voltageMv); }

void runIntakeIn() { spinIntake(kFullPowerMv); }

void runOuttake() { spinIntake(-kFullPowerMv); }

void runLowGoalScore() {
  setIntakeLiftUp(true);
  spinIntake(-kLowGoalMv);
}

void runLowGoalScoreFor(std::uint32_t durationMs) {
  runLowGoalScore();
  pros::delay(durationMs);
  setIntakeLiftUp(false);
  stopIntake();
}

bool isAnyIntakeRunning() { return intakeRunning; }

bool isLeverRunning() { return leverRunning; }

void setMatchLoaderUp(bool up) { matchLoader.set_value(up); }

void setIntakeLiftUp(bool up) { intakeLift.set_value(up); }

void setWingFourBarUp(bool up) { wingFourBar.set_value(up); }

void setWingUp(bool up, pros::adi::DigitalOut wing) { wing.set_value(up); }

void setHoodUp(bool up) { hood.set_value(up); }

// void setLeverPistonUp(bool up) { leverPiston.set_value(up); }

void setBasketUp(bool up) { basket.set_value(up); }

}  // namespace robotactions
