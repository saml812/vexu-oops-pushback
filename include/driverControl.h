#ifndef DRIVER_CONTROL_H_
#define DRIVER_CONTROL_H_

#include "main.h"

void runDriveControl();
void runScoringControl();
void runWingHoldControl();
void runR1IntakeControl();
void runL1LeverControl();
void runDownIntakeTimedControl();
void runOuttakeControl();
void runLowGoalControl();
void runIntakeLiftOutControl();
void runMatchLoaderHoldControl();
void runWingFourBarToggleControl();
void runInertialRecalibrationControl();
void runDriverControl();
void runBasketControl();
void runWingAlignControl();

bool isMatchLoaderDown();
bool isIntakeLiftUp();
bool isWingFourBarUp();
bool isWingUp();
bool isMidGoalUp();
bool isHoodUp();
bool isLeverUp();
bool isBasketUp();
bool isLeverArmDown();

void vibrateController(const char* pattern);

int getFullIntakeVoltage();
int getPartialOuttakeVoltage();

#endif
