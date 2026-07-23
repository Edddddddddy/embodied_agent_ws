#include "embodied_slam/closed_loop_controller.hpp"

#include <cmath>
#include <utility>

namespace embodied_slam
{

namespace
{
constexpr double kQuarterTurn = 1.57079632679489661923;
}

ClosedLoopController::ClosedLoopController(ClosedLoopConfig config)
: config_(std::move(config))
{
}

DriveCommand ClosedLoopController::update(const Pose2d & pose)
{
  if (state_ == State::kWaiting) {
    state_start_pose_ = pose;
    previous_pose_ = pose;
    state_ = State::kDriving;
  }

  if (state_ == State::kComplete) {
    return DriveCommand{0.0, 0.0, true};
  }

  if (state_ == State::kDriving) {
    if (distance(state_start_pose_, pose) + config_.position_tolerance_m <
      config_.segment_length_m)
    {
      previous_pose_ = pose;
      return DriveCommand{config_.linear_speed_mps, 0.0, false};
    }
    state_ = State::kTurning;
    accumulated_turn_rad_ = 0.0;
    previous_pose_ = pose;
  }

  if (state_ == State::kTurning) {
    // 累加相邻姿态差而不是直接比较绝对 yaw，保证跨越 ±pi 时仍能正确完成 90° 转向。
    accumulated_turn_rad_ += std::abs(normalize_angle(pose.yaw - previous_pose_.yaw));
    previous_pose_ = pose;
    if (accumulated_turn_rad_ + config_.angle_tolerance_rad < kQuarterTurn) {
      return DriveCommand{0.0, config_.angular_speed_rps, false};
    }
    ++completed_sides_;
    if (completed_sides_ >= 4U * config_.loop_count) {
      state_ = State::kComplete;
      return DriveCommand{0.0, 0.0, true};
    }
    state_start_pose_ = pose;
    state_ = State::kDriving;
    return DriveCommand{config_.linear_speed_mps, 0.0, false};
  }

  return DriveCommand{};
}

void ClosedLoopController::reset()
{
  state_ = State::kWaiting;
  state_start_pose_ = {};
  previous_pose_ = {};
  accumulated_turn_rad_ = 0.0;
  completed_sides_ = 0U;
}

std::size_t ClosedLoopController::completed_sides() const
{
  return completed_sides_;
}

}  // namespace embodied_slam
