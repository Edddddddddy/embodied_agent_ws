#include "embodied_simulation/simulation_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace embodied_simulation
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kFrontHalfWidth = 20.0 * kPi / 180.0;
constexpr double kSideHalfWidth = 25.0 * kPi / 180.0;
}

SimulationController::SimulationController(ControllerConfig config)
: config_(std::move(config))
{
}

bool SimulationController::set_mode(const std::string & mode)
{
  ControlMode requested;
  if (mode == "manual") {
    requested = ControlMode::kManual;
  } else if (mode == "obstacle_avoidance") {
    requested = ControlMode::kObstacleAvoidance;
  } else if (mode == "wall_following") {
    requested = ControlMode::kWallFollowing;
  } else {
    return false;
  }
  mode_ = requested;
  manual_command_ = {};
  manual_until_s_ = 0.0;
  force_stop_ = true;
  reset_pid();
  return true;
}

ControlMode SimulationController::mode() const
{
  return mode_;
}

std::string SimulationController::mode_name(ControlMode mode)
{
  switch (mode) {
    case ControlMode::kManual:
      return "manual";
    case ControlMode::kObstacleAvoidance:
      return "obstacle_avoidance";
    case ControlMode::kWallFollowing:
      return "wall_following";
  }
  return "unknown";
}

void SimulationController::set_manual_command(
  double linear_x, double angular_z, double duration_s, double now_s)
{
  mode_ = ControlMode::kManual;
  manual_command_.linear_x = clamp(
    linear_x, -config_.max_linear_speed, config_.max_linear_speed);
  manual_command_.angular_z = clamp(
    angular_z, -config_.max_angular_speed, config_.max_angular_speed);
  manual_until_s_ = now_s + clamp(duration_s, 0.0, 10.0);
  force_stop_ = false;
  reset_pid();
}

void SimulationController::stop()
{
  mode_ = ControlMode::kManual;
  manual_command_ = {};
  manual_until_s_ = 0.0;
  current_command_ = {};
  force_stop_ = true;
}

void SimulationController::set_emergency_stop(bool active)
{
  emergency_stop_ = active;
  if (active) {
    stop();
  }
}

void SimulationController::update_scan(
  const std::vector<float> & ranges,
  double angle_min,
  double angle_increment,
  double range_min,
  double range_max,
  double now_s)
{
  double front = std::numeric_limits<double>::infinity();
  double left = std::numeric_limits<double>::infinity();
  double right = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < ranges.size(); ++index) {
    const double range = ranges[index];
    if (!std::isfinite(range) || range < range_min || range > range_max) {
      continue;
    }
    const double angle = normalized_angle(
      angle_min + static_cast<double>(index) * angle_increment);
    if (in_sector(angle, 0.0, kFrontHalfWidth)) {
      front = std::min(front, range);
    }
    if (in_sector(angle, kPi / 2.0, kSideHalfWidth)) {
      left = std::min(left, range);
    }
    if (in_sector(angle, -kPi / 2.0, kSideHalfWidth)) {
      right = std::min(right, range);
    }
  }
  front_distance_ = front;
  left_distance_ = left;
  right_distance_ = right;
  last_scan_s_ = now_s;
}

ControllerOutput SimulationController::step(double now_s)
{
  double dt = 0.05;
  if (std::isfinite(last_step_s_) && now_s >= last_step_s_) {
    dt = clamp(now_s - last_step_s_, 0.001, 0.20);
  }
  last_step_s_ = now_s;

  const bool scan_stale = now_s - last_scan_s_ > config_.scan_timeout;
  VelocityCommand target;
  std::string reason = "manual_idle";

  if (emergency_stop_) {
    reason = "emergency_stop";
  } else if (mode_ == ControlMode::kManual) {
    if (now_s <= manual_until_s_) {
      target = manual_command_;
      reason = "manual_command";
    } else {
      reason = "manual_timeout";
    }
  } else if (scan_stale) {
    reason = "scan_timeout";
  } else if (mode_ == ControlMode::kObstacleAvoidance) {
    if (front_distance_ < config_.obstacle_distance) {
      target.angular_z = left_distance_ >= right_distance_ ?
        config_.obstacle_turn_speed : -config_.obstacle_turn_speed;
      reason = "obstacle_turn";
    } else {
      target.linear_x = config_.autonomous_linear_speed;
      reason = "obstacle_clear";
    }
  } else if (mode_ == ControlMode::kWallFollowing) {
    if (front_distance_ < config_.obstacle_distance) {
      target.angular_z = config_.obstacle_turn_speed;
      reason = "wall_corner";
      reset_pid();
    } else if (!std::isfinite(right_distance_)) {
      target.linear_x = config_.autonomous_linear_speed * 0.6;
      target.angular_z = -config_.obstacle_turn_speed * 0.5;
      reason = "wall_search";
    } else {
      const double error = config_.wall_target_distance - right_distance_;
      wall_integral_ = clamp(wall_integral_ + error * dt, -0.5, 0.5);
      const double derivative = (error - wall_previous_error_) / dt;
      wall_previous_error_ = error;
      target.linear_x = config_.autonomous_linear_speed;
      target.angular_z = clamp(
        config_.wall_kp * error + config_.wall_ki * wall_integral_ +
        config_.wall_kd * derivative,
        -config_.max_angular_speed, config_.max_angular_speed);
      reason = "wall_tracking";
    }
  }

  bool safety_stopped = false;
  if (scan_stale && target.linear_x > 0.0) {
    target.linear_x = 0.0;
    safety_stopped = true;
    reason = "scan_timeout";
  } else if (!scan_stale && front_distance_ < config_.emergency_distance &&
    target.linear_x > 0.0)
  {
    target.linear_x = 0.0;
    safety_stopped = true;
    reason = "front_emergency";
  }

  target.linear_x = clamp(
    target.linear_x, -config_.max_linear_speed, config_.max_linear_speed);
  target.angular_z = clamp(
    target.angular_z, -config_.max_angular_speed, config_.max_angular_speed);

  if (force_stop_ || emergency_stop_ ||
    (scan_stale && mode_ != ControlMode::kManual) || safety_stopped)
  {
    current_command_ = target;
    force_stop_ = false;
  } else {
    current_command_.linear_x = approach(
      current_command_.linear_x, target.linear_x, config_.linear_acceleration * dt);
    current_command_.angular_z = approach(
      current_command_.angular_z, target.angular_z, config_.angular_acceleration * dt);
  }

  return ControllerOutput{
    current_command_, mode_, scan_stale, safety_stopped,
    front_distance_, right_distance_, reason};
}

double SimulationController::approach(double current, double target, double max_delta)
{
  if (current < target) {
    return std::min(current + max_delta, target);
  }
  return std::max(current - max_delta, target);
}

double SimulationController::normalized_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

bool SimulationController::in_sector(double angle, double center, double half_width)
{
  return std::abs(normalized_angle(angle - center)) <= half_width;
}

double SimulationController::clamp(double value, double lower, double upper)
{
  return std::clamp(value, lower, upper);
}

void SimulationController::reset_pid()
{
  wall_integral_ = 0.0;
  wall_previous_error_ = 0.0;
}

}  // namespace embodied_simulation
