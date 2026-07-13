#include "embodied_slam/trajectory_metrics.hpp"

#include <algorithm>
#include <cmath>

namespace embodied_slam
{

namespace
{
double path_length(const std::vector<Pose2d> & path, std::size_t count)
{
  double total = 0.0;
  for (std::size_t index = 1U; index < count; ++index) {
    total += distance(path[index - 1U], path[index]);
  }
  return total;
}
}  // namespace

TrajectoryMetrics evaluate_trajectory(
  const std::vector<Pose2d> & reference,
  const std::vector<Pose2d> & estimate)
{
  TrajectoryMetrics metrics;
  const std::size_t count = std::min(reference.size(), estimate.size());
  metrics.sample_count = count;
  if (count == 0U) {
    return metrics;
  }

  metrics.reference_path_length_m = path_length(reference, count);
  metrics.estimate_path_length_m = path_length(estimate, count);
  double squared_error_sum = 0.0;
  for (std::size_t index = 0U; index < count; ++index) {
    const double error = distance(reference[index], estimate[index]);
    squared_error_sum += error * error;
  }
  metrics.ate_rmse_m = std::sqrt(squared_error_sum / static_cast<double>(count));
  metrics.final_position_error_m = distance(reference[count - 1U], estimate[count - 1U]);
  metrics.final_yaw_error_rad = std::abs(normalize_angle(
      estimate[count - 1U].yaw - reference[count - 1U].yaw));
  metrics.closure_error_m = distance(estimate.front(), estimate[count - 1U]);
  return metrics;
}

}  // namespace embodied_slam
