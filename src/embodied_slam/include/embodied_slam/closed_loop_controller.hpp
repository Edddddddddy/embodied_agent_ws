#pragma once

#include <cstddef>

#include "embodied_slam/pose2d.hpp"

namespace embodied_slam
{

struct ClosedLoopConfig
{
  double segment_length_m{2.0};
  double linear_speed_mps{0.18};
  double angular_speed_rps{0.45};
  double position_tolerance_m{0.04};
  double angle_tolerance_rad{0.035};
  std::size_t loop_count{1U};
};

struct DriveCommand
{
  double linear_x{0.0};
  double angular_z{0.0};
  bool complete{false};
};

class ClosedLoopController
{
public:
  explicit ClosedLoopController(ClosedLoopConfig config = {});

  DriveCommand update(const Pose2d & pose);
  void reset();
  std::size_t completed_sides() const;

private:
  enum class State {kWaiting, kDriving, kTurning, kComplete};

  ClosedLoopConfig config_;
  State state_{State::kWaiting};
  Pose2d state_start_pose_;
  Pose2d previous_pose_;
  double accumulated_turn_rad_{0.0};
  std::size_t completed_sides_{0U};
};

}  // namespace embodied_slam
