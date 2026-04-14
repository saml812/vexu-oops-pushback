#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include <cstdint>

#include "lemlib/api.hpp"

enum class AutonRoutine : std::uint8_t { DoNothing = 0, Left, Right };

const char* autonRoutineName(AutonRoutine routine);
void runAutonRoutine(AutonRoutine routine);
void moveDistance(float distance, int timeout,
                  lemlib::MoveToPointParams params = {}, bool async = true);

#endif
