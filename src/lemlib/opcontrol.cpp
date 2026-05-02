#include "lemlib/opcontrol.h"

#include <math.h>

#include <algorithm>

extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
extern ez::Drive chassis;

namespace lemlib {
ExpoDriveCurve defaultDriveCurve = ExpoDriveCurve(0, 0, 1);

void tank(int left, int right, bool disableDriveCurve) {
  if (disableDriveCurve) {
    chassis.drive_set(left, right);
  } else {
    chassis.drive_set(throttleCurve.curve(left), throttleCurve.curve(right));
  }
}

void arcade(int throttle, int turn, bool disableDriveCurve, float desaturateBias) {
  // use drive curves if they have not been disabled
  if (!disableDriveCurve) {
    throttle = std::round(throttleCurve.curve(throttle));
    turn = std::round(steerCurve.curve(turn));
  }
  // desaturate motors based on joyBias
  if (std::abs(throttle) + std::abs(turn) > 127) {
    int oldThrottle = throttle;
    int oldTurn = turn;
    throttle *= (1 - desaturateBias * std::abs(oldTurn / 127.0));
    turn *= (1 - (1 - desaturateBias) * std::abs(oldThrottle / 127.0));
    // ensure the sum of the two values is equal to 127
    // this check is necessary because of integer division
    if (std::abs(turn) + std::abs(throttle) == 126) {
      if (desaturateBias < 0.5)
        throttle += sgn(throttle);
      else
        turn += sgn(turn);
    }
  }

  int leftPower = throttle + turn;
  int rightPower = throttle - turn;

  // move drive
  leftDrive.move(leftPower);
  rightDrive.move(rightPower);

//   chassis.drive_set(leftPower, rightPower);
}

void curvature(int throttle, int turn, bool disableDriveCurve) {
  // If we're not moving forwards change to arcade drive
  if (throttle == 0) {
    arcade(throttle, turn, disableDriveCurve);
    return;
  }

  // use drive curves if they have not been disabled
  if (!disableDriveCurve) {
    throttle = throttleCurve.curve(throttle);
    turn = steerCurve.curve(turn);
  }

  float leftPower = throttle + (std::fabs(throttle) * turn / 127.0);
  float rightPower = throttle - (std::fabs(throttle) * turn / 127.0);

  // desaturate output
  float max = std::max(std::fabs(leftPower), std::fabs(rightPower)) / 127;
  if (max > 1) {
    leftPower /= max;
    rightPower /= max;
  }
  chassis.drive_set(leftPower, rightPower);
}
}  // namespace lemlib