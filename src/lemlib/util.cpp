#include <vector>
#include "lemlib/util.hpp"

float lemlib::slew(float target, float current, float maxChange) {
    float change = target - current;
    if (maxChange == 0) return target;
    if (change > maxChange) change = maxChange;
    else if (change < -maxChange) change = -maxChange;
    return current + change;
}

constexpr float lemlib::sanitizeAngle(float angle, bool radians) {
    if (radians) return std::fmod(std::fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    else return std::fmod(std::fmod(angle, 360) + 360, 360);
}

float lemlib::avg(std::vector<float> values) {
    float sum = 0;
    for (float value : values) { sum += value; }
    return sum / values.size();
}

float lemlib::ema(float current, float previous, float smooth) {
    return (current * smooth) + (previous * (1 - smooth));
}
