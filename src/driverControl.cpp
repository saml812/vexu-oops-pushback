#include "driverControl.h"

#include <array>
#include <cstdint>

#include "configure.h"
#include "robotActions.h"

namespace {
constexpr std::uint32_t TOP_ROLLER_REVERSAL_MS = 125;

constexpr std::uint32_t DEFAULT_DEBOUNCE_MS = 120;
constexpr std::uint32_t TOGGLE_DEBOUNCE_MS = 220;
constexpr std::uint32_t DOUBLE_TAP_MS = 200;

constexpr double MAX_ANGLE = 115.0;
constexpr double MIN_ANGLE = 0.0;
constexpr double TOLERANCE = 3.0;
constexpr double SKILLS_SPEED = 5400;
constexpr double SKILLS_SPEED_OUTAKE = 4800;

bool matchLoaderUp = true;
bool intakeLiftUp = false;
// bool wing4BarUp = false;
bool wingUp = true;
bool hoodUp = false;
bool downIntakeActive = false;
std::uint32_t downIntakeStartMs = 0;
bool intakeDriveCutActive = false;

bool basketUp = false;
// bool leverPistonUp = false;
bool leverArmDown = false;

pros::controller_digital_e_t lastPowerButtonPressed;
bool aPressed = false;
bool leftPressed = false;

std::array<std::uint32_t, 32> lastButtonPressTimeMs = {0};

pros::v5::Motor& drivetrainPort12Motor() {
  static pros::v5::Motor motor(12, pros::v5::MotorGears::blue,
                               pros::v5::MotorUnits::deg);
  return motor;
}

pros::v5::Motor& drivetrainPort20Motor() {
  static pros::v5::Motor motor(-20, pros::v5::MotorGears::blue,
                               pros::v5::MotorUnits::deg);
  return motor;
}

void applyWingHold(bool held, pros::adi::DigitalOut wing) {
  wingUp = !held;
  robotactions::setWingUp(wingUp, wing);
}

void applyIntakeDriveMotorCut() {
  const bool cutNeeded = robotactions::isAnyIntakeRunning() || robotactions::isLeverRunning();

  auto& port12 = drivetrainPort12Motor();
  auto& port20 = drivetrainPort20Motor();

  if (cutNeeded) {
    if (!intakeDriveCutActive) {
      port12.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      port20.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      intakeDriveCutActive = true;
    }

    port12.move_voltage(0);
    port20.move_voltage(0);
    return;
  }

  if (intakeDriveCutActive) {
    port12.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    port20.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    intakeDriveCutActive = false;
  }
}

bool isDoubleTap(pros::controller_digital_e_t button,
                 std::uint32_t doubleTapMs = DOUBLE_TAP_MS) {
  if (!master.get_digital_new_press(button)) {
    return false;
  }

  static std::array<std::uint32_t, 32> lastDoubleTapTimeMs = {0};

  const auto nowMs = static_cast<std::uint32_t>(pros::millis());
  const auto index = static_cast<std::size_t>(button);

  if (index >= lastDoubleTapTimeMs.size()) {
    return false;
  }

  std::uint32_t& lastTime = lastDoubleTapTimeMs[index];

  if (lastTime != 0U && (nowMs - lastTime) <= doubleTapMs) {
    lastTime = 0U;
    return true;
  }

  lastTime = nowMs;
  return false;
}

bool isDebouncedNewPress(pros::controller_digital_e_t button,
                         std::uint32_t debounceMs = DEFAULT_DEBOUNCE_MS) {
  if (!master.get_digital_new_press(button)) {
    return false;
  }

  const auto nowMs = static_cast<std::uint32_t>(pros::millis());
  const auto index = static_cast<std::size_t>(button);

  if (index < lastButtonPressTimeMs.size() &&
      lastButtonPressTimeMs[index] != 0U &&
      (nowMs - lastButtonPressTimeMs[index]) < debounceMs) {
    return false;
  }

  if (index < lastButtonPressTimeMs.size()) {
    lastButtonPressTimeMs[index] = nowMs;
  }

  return true;
}
}  // namespace

void runScoringControl() {
  hoodUp = true;
  robotactions::setHoodUp(true);
  // robotactions::spinAllIntake(robotactions::kFullPowerMv);
  // robotactions::spinLiftMotors(robotactions::kFullPowerMv);
}

void runWingHoldControl() {
  bool downPressed = master.get_digital(DIGITAL_DOWN);
  applyWingHold(downPressed, wingR);

  bool bPressed = master.get_digital(DIGITAL_B);
  applyWingHold(bPressed, wingL);
}

void runIntakeControl() {
  if (master.get_digital(DIGITAL_R1)) {
    robotactions::setIntakeLiftUp(false);
    robotactions::spinIntake((robotactions::kFullPowerMv));
  }
}

void runL1LeverControl() {
  enum LeverState { MOVING_UP,
                    MOVING_DOWN };
  static LeverState state = MOVING_DOWN;
  bool extendPiston = false;

  int power = (basketUp ? robotactions::kFullPowerMv : robotactions::kFullPowerMv / 2);

  //   int power = SKILLS_SPEED;

  if (lastPowerButtonPressed == DIGITAL_A && aPressed) {
    power = robotactions::kFullPowerMv;
    extendPiston = true;
  } else if (lastPowerButtonPressed == DIGITAL_LEFT && leftPressed) {
    power /= 2;
    extendPiston = false;
  }

  double currentAngle = lever.get_position();
  if (currentAngle < 0) {
    lever.set_zero_position(0.0);
    currentAngle = 0.0;
  }

  bool l1Pressed = master.get_digital(DIGITAL_L1);

  if (l1Pressed) {
    state = MOVING_UP;
    robotactions::setHoodUp(true);
    hoodUp = true;
  } else {
    state = MOVING_DOWN;
    robotactions::setHoodUp(false);
    hoodUp = false;
  }

  switch (state) {
    case MOVING_UP:
      //   if (currentAngle >= MAX_ANGLE - TOLERANCE) {
      //     robotactions::stopLever();
      //   } else {
      //     robotactions::spinLever(power);
      //   }
      pros::delay(200);
      robotactions::spinLever(power);

      //   robotactions::setLeverPistonUp(extendPiston);
      extendPiston = false;
      break;
    case MOVING_DOWN:
      if (currentAngle <= MIN_ANGLE + TOLERANCE) {
        robotactions::stopLever();
      } else {
        robotactions::spinLever(-robotactions::kFullPowerMv / 4);
      }

      //   robotactions::setLeverPistonUp(false);
      break;
  }
}

void runDownIntakeTimedControl() {
  if (!master.get_digital(DIGITAL_DOWN)) {
    downIntakeActive = false;
    return;
  }

  if (!downIntakeActive) {
    downIntakeActive = true;
    downIntakeStartMs = static_cast<std::uint32_t>(pros::millis());
  }

  const std::uint32_t elapsedMs =
      static_cast<std::uint32_t>(pros::millis()) - downIntakeStartMs;
  const int topRollerVoltage = elapsedMs < TOP_ROLLER_REVERSAL_MS
                                   ? -robotactions::kFullPowerMv
                                   : robotactions::kFullPowerMv;
  const bool midGoalShouldBeUp =
      elapsedMs < TOP_ROLLER_REVERSAL_MS ? true : false;
  robotactions::setIntakeVoltage(-robotactions::kFullPowerMv);
}

void runOuttakeControl() {
  if (master.get_digital(DIGITAL_R2)) {
    robotactions::runOuttake();
  }
}

void runIntakeLiftOutControl() {
  if (isDoubleTap(DIGITAL_R2)) {
    robotactions::setIntakeLiftUp(true);
  }
}

void runLowGoalControl() {
  const bool pressed = master.get_digital(DIGITAL_Y);
  intakeLiftUp = !pressed;
  robotactions::setIntakeLiftUp(intakeLiftUp);

  if (pressed) {
    // robotactions::runLowGoalScore();
  }
}

void runMatchLoaderToggleControl() {
  if (isDebouncedNewPress(DIGITAL_Y, TOGGLE_DEBOUNCE_MS)) {
    matchLoaderUp = !matchLoaderUp;
    robotactions::setMatchLoaderUp(matchLoaderUp);
  }
}

// void runWingFourBarToggleControl() {
//   if (isDebouncedNewPress(DIGITAL_Y, TOGGLE_DEBOUNCE_MS)) {
//     wing4BarUp = !wing4BarUp;
//   }

//   robotactions::setWingFourBarUp(wing4BarUp);
// }

void runInertialRecalibrationControl() {
  if (!isDebouncedNewPress(DIGITAL_UP, 500)) {
    return;
  }

  if (!inertial.is_calibrating()) {
    inertial.reset(false);
  }
}

void runBasketControl() {
  if (isDebouncedNewPress(DIGITAL_RIGHT, TOGGLE_DEBOUNCE_MS)) {
    basketUp = !basketUp;
    robotactions::setBasketUp(basketUp);
  }
}

void runWingAlignControl() {
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.pid_drive_set(12_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(315_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait_quick_chain();
}

void runDriverControl() {
  //   runInertialRecalibrationControl();
  runWingHoldControl();
  runMatchLoaderToggleControl();
  //   runWingFourBarToggleControl();
  runBasketControl();
  runL1LeverControl();
  runIntakeLiftOutControl();

  robotactions::stopIntake();

  if (master.get_digital(DIGITAL_R2)) {
    downIntakeActive = false;
    runOuttakeControl();
  } else if (master.get_digital(DIGITAL_R1)) {
    downIntakeActive = false;
    runIntakeControl();
  } else if (isDebouncedNewPress(DIGITAL_LEFT, TOGGLE_DEBOUNCE_MS)) {
    if (lastPowerButtonPressed != DIGITAL_LEFT) {
      vibrateController("--");
    } else {
      vibrateController(".");
    }
    lastPowerButtonPressed = DIGITAL_LEFT;
    leftPressed = !leftPressed;
  } else if (isDebouncedNewPress(DIGITAL_A, TOGGLE_DEBOUNCE_MS)) {
    if (lastPowerButtonPressed != DIGITAL_A) {
      vibrateController("--");
    } else {
      vibrateController(".");
    }
    aPressed = !aPressed;
    lastPowerButtonPressed = DIGITAL_A;
  } else {
    downIntakeActive = false;
  }

  applyIntakeDriveMotorCut();
}

bool ismatchLoaderUp() { return matchLoaderUp; }

bool isIntakeLiftUp() { return intakeLiftUp; }

// bool isWingFourBarUp() { return wing4BarUp; }

bool isWingUp() { return wingUp; }

bool isHoodUp() { return hoodUp; }

bool isBasketUp() { return basketUp; }

// bool isLeverPistonUp() { return leverPistonUp; }

bool isLeverArmDown() {
  return lever.get_position() <= MIN_ANGLE + TOLERANCE;
}

int getFullIntakeVoltage() { return robotactions::kFullPowerMv; }

int getPartialOuttakeVoltage() { return -robotactions::kPartialOuttakeMv; }

void vibrateController(const char* pattern) {
  master.rumble(pattern);
}