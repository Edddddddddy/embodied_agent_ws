#pragma once

#include <cstddef>
#include <vector>

#include "embodied_slam/pose2d.hpp"

namespace embodied_slam
{

struct TrajectoryMetrics
{
  std::size_t sample_count{0U};
  double reference_path_length_m{0.0};
  double estimate_path_length_m{0.0};
  double ate_rmse_m{0.0};
  double final_position_error_m{0.0};
  double final_yaw_error_rad{0.0};
  double closure_error_m{0.0};
};

TrajectoryMetrics evaluate_trajectory(
  const std::vector<Pose2d> & reference,
  const std::vector<Pose2d> & estimate);

}  // namespace embodied_slam
