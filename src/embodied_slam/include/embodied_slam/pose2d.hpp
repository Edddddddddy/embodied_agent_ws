#pragma once

#include <cmath>

namespace embodied_slam
{

struct Pose2d
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double normalize_angle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

inline double distance(const Pose2d & lhs, const Pose2d & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

}  // namespace embodied_slam
