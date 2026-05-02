#pragma once

#include "EZ-Template/api.hpp"
#include "lemlib/driveCurve.h"
#include "lemlib/util.hpp"
#include "configure.h"

namespace lemlib {
void tank(int left, int right, bool disableDriveCurve = false);
void arcade(int throttle, int turn, bool disableDriveCurve = false, float desaturateBias = 0.5);
void curvature(int throttle, int turn, bool disableDriveCurve = false);
}