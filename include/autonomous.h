#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include "lemlib/api.hpp"

#include <cstdint>

enum class AutonRoutine : std::uint8_t {
    DoNothing = 0,
    Left,
    Right
};

const char* autonRoutineName(AutonRoutine routine);
void runAutonRoutine(AutonRoutine routine);
void moveDistance(float distance, int timeout, lemlib::MoveToPointParams params = {}, bool async = true);

#endif
