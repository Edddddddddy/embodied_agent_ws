#include <gtest/gtest.h>

#include <vector>

#include "embodied_slam/trajectory_metrics.hpp"

namespace embodied_slam
{

TEST(TrajectoryMetrics, ReportsExactMatchingTrajectory)
{
  const std::vector<Pose2d> path = {
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 1.57}, {0.0, 0.0, 3.14}};
  const auto metrics = evaluate_trajectory(path, path);
  EXPECT_EQ(metrics.sample_count, path.size());
  EXPECT_DOUBLE_EQ(metrics.ate_rmse_m, 0.0);
  EXPECT_DOUBLE_EQ(metrics.final_position_error_m, 0.0);
  EXPECT_DOUBLE_EQ(metrics.closure_error_m, 0.0);
}

TEST(TrajectoryMetrics, MeasuresDriftAndClosureError)
{
  const std::vector<Pose2d> reference = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  const std::vector<Pose2d> estimate = {{0.0, 0.0, 0.0}, {1.2, 0.0, 0.1}, {0.3, 0.4, 0.2}};
  const auto metrics = evaluate_trajectory(reference, estimate);
  EXPECT_NEAR(metrics.closure_error_m, 0.5, 1e-9);
  EXPECT_NEAR(metrics.final_position_error_m, 0.5, 1e-9);
  EXPECT_NEAR(metrics.final_yaw_error_rad, 0.2, 1e-9);
  EXPECT_GT(metrics.ate_rmse_m, 0.0);
}

TEST(TrajectoryMetrics, UsesOnlyPairedSamples)
{
  const std::vector<Pose2d> reference = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  const std::vector<Pose2d> estimate = {{0.0, 0.0, 0.0}};
  const auto metrics = evaluate_trajectory(reference, estimate);
  EXPECT_EQ(metrics.sample_count, 1U);
  EXPECT_DOUBLE_EQ(metrics.reference_path_length_m, 0.0);
}

}  // namespace embodied_slam
