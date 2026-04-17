#include "robotActions.h"

#include <cstdint>

#include "configure.h"

namespace {
bool intakeRunning = false;
bool leverRunning = false;
}  // namespace

namespace robotactions {
void setIntakeVoltages(int voltageMv) {
    leftIntake.move_voltage(voltageMv);
    rightIntake.move_voltage(voltageMv);

    // intake.move_voltage(voltageMv);
    intakeRunning = voltageMv != 0;
}

void setLeverVoltages(int voltageMv) {
    // leftLever.move_voltage(voltageMv);
    // rightLever.move_voltage(voltageMv);

    lever.move_voltage(voltageMv);

    leverRunning = voltageMv != 0;
}

void stopAllIntake() { setIntakeVoltages(0); }

void stopLever() { setLeverVoltages(0); }

void spinAllIntake(int voltageMv) { setIntakeVoltages(voltageMv); }

void spinLever(int voltageMv) { setLeverVoltages(voltageMv); }

void runIntakeIn() { spinAllIntake(kFullPowerMv); }

void runOuttake() { spinAllIntake(-kFullPowerMv); }

void runLowGoalScore() {
    setIntakeLiftUp(true);
    spinAllIntake(-kLowGoalMv);
}

void runLowGoalScoreFor(std::uint32_t durationMs) {
    runLowGoalScore();
    pros::delay(durationMs);
    setIntakeLiftUp(false);
    stopAllIntake();
}

bool isAnyIntakeRunning() { return intakeRunning; }

bool isLeverRunning() { return leverRunning; }

void setMatchLoaderDown(bool down) { matchLoader.set_value(down); }

void setIntakeLiftUp(bool up) { intakeLift.set_value(up); }

void setWingFourBarUp(bool up) { wingFourBar.set_value(up); }

void setWingUp(bool up) { wing.set_value(up); }

void setMidGoalUp(bool up) { midGoal.set_value(up); }

void setHoodUp(bool up) { hood.set_value(up); }

void setLeverPistonUp(bool up) { leverPiston.set_value(up); }

void setBasketUp(bool up) { basket.set_value(up); }

}  // namespace robotactions
