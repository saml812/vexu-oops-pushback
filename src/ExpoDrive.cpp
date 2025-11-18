#include "ExpoDrive.hpp"

#include <algorithm>
#include <cmath>

ExpoDrive::ExpoDrive(double a, double d, double s, double n)
    : a_(std::clamp(a, 1.0, 1.3)),
      d_(std::clamp(d, 0.0, s)),
      s_(std::max(1.0, s)),
      n_(std::clamp(n, 0.0, s)) {}

double ExpoDrive::calculateG(double x) const {
  return std::abs(x) - d_;
}

double ExpoDrive::calculateI(double x) const {
  if (x == 0.0) return 0.0;

  double g_x = calculateG(x);
  if (g_x <= 0) return 0.0;

  int sign = (x > 0) ? 1 : -1;
  double exponent = g_x - s_;
  return std::pow(a_, exponent) * g_x * sign;
}

double ExpoDrive::calculate(double x) const {
  // Equation 3: j(x) = 0 for -d < x < d
  if (std::abs(x) < d_) {
    return 0.0;
  }

  // Clamp x to range [-s, s]
  double clamped_x = std::clamp(x, -s_, s_);

  // Equation 4: k(x)
  if (clamped_x == 0.0) return 0.0;

  int sign = (clamped_x > 0) ? 1 : -1;
  double i_x = calculateI(clamped_x);
  double i_s = calculateI(s_ * sign);

  if (i_s == 0.0) {
    return n_ * sign;
  }

  double scale_factor = (s_ - n_) / s_;
  return scale_factor * (i_x * s_ / i_s) + n_ * sign;
}

void ExpoDrive::setA(double a) {
  a_ = std::clamp(a, 1.0, 1.3);
}

void ExpoDrive::setD(double d) {
  d_ = std::clamp(d, 0.0, s_);
}

void ExpoDrive::setS(double s) {
  s_ = std::max(1.0, s);
  d_ = std::clamp(d_, 0.0, s_);
  n_ = std::clamp(n_, 0.0, s_);
}

void ExpoDrive::setN(double n) {
  n_ = std::clamp(n, 0.0, s_);
}