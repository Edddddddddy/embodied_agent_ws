#include <gtest/gtest.h>

#include <unordered_map>
#include <vector>

#include "embodied_slam/gtsam_pose_graph.hpp"

namespace embodied_slam
{

PoseGraphConstraint between(int source, int target, double x, double y = 0.0, double yaw = 0.0)
{
  PoseGraphConstraint constraint;
  constraint.source_id = source;
  constraint.target_id = target;
  constraint.relative_pose = {x, y, yaw};
  constraint.covariance = Eigen::Matrix3d::Identity() * 0.01;
  return constraint;
}

TEST(GtsamPoseGraphOptimizer, CorrectsAccumulatedDriftWithLoopConstraint)
{
  const std::unordered_map<int, Pose2d> initial = {
    {0, {0.0, 0.0, 0.0}}, {1, {1.1, 0.0, 0.0}}, {2, {2.2, 0.0, 0.0}}};
  const std::vector<PoseGraphConstraint> constraints = {
    between(0, 1, 1.0), between(1, 2, 1.0), between(0, 2, 2.0)};
  const auto result = GtsamPoseGraphOptimizer().optimize(initial, constraints);
  ASSERT_EQ(result.poses.size(), 3U);
  EXPECT_NEAR(result.poses.at(2).x, 2.0, 1e-4);
  EXPECT_LT(result.final_error, result.initial_error);
}

TEST(GtsamPoseGraphOptimizer, AnchorsTheLowestScanId)
{
  const std::unordered_map<int, Pose2d> initial = {
    {7, {4.0, -2.0, 0.3}}, {8, {5.2, -1.6, 0.3}}};
  const auto result = GtsamPoseGraphOptimizer().optimize(initial, {between(7, 8, 1.0)});
  EXPECT_NEAR(result.poses.at(7).x, 4.0, 1e-6);
  EXPECT_NEAR(result.poses.at(7).y, -2.0, 1e-6);
  EXPECT_NEAR(result.poses.at(7).yaw, 0.3, 1e-6);
}

TEST(GtsamPoseGraphOptimizer, SanitizesSingularScanMatchingCovariance)
{
  PoseGraphConstraint constraint = between(0, 1, 1.0);
  constraint.covariance.setZero();
  const std::unordered_map<int, Pose2d> initial = {
    {0, {0.0, 0.0, 0.0}}, {1, {1.1, 0.0, 0.0}}};
  EXPECT_NO_THROW({
    const auto result = GtsamPoseGraphOptimizer().optimize(initial, {constraint});
    EXPECT_NEAR(result.poses.at(1).x, 1.0, 1e-4);
  });
}

TEST(GtsamPoseGraphOptimizer, ParsesSupportedRobustKernels)
{
  EXPECT_EQ(robust_kernel_from_string("NONE"), RobustKernel::kNone);
  EXPECT_EQ(robust_kernel_from_string("Huber"), RobustKernel::kHuber);
  EXPECT_EQ(robust_kernel_from_string("cauchy"), RobustKernel::kCauchy);
  EXPECT_THROW(robust_kernel_from_string("magic"), std::invalid_argument);
}

TEST(GtsamPoseGraphOptimizer, CauchyLoopOnlySuppressesAFalseLongRangeClosure)
{
  std::unordered_map<int, Pose2d> initial;
  std::vector<PoseGraphConstraint> constraints;
  for (int id = 0; id <= 10; ++id) {
    initial[id] = {static_cast<double>(id), 0.0, 0.0};
    if (id > 0) {
      constraints.push_back(between(id - 1, id, 1.0));
    }
  }
  auto false_loop = between(0, 10, 0.0);
  false_loop.covariance = Eigen::Matrix3d::Identity() * 0.0001;
  constraints.push_back(false_loop);

  PoseGraphOptimizerConfig gaussian_config;
  gaussian_config.robust_kernel = RobustKernel::kNone;
  const auto gaussian = GtsamPoseGraphOptimizer(gaussian_config).optimize(initial, constraints);

  PoseGraphOptimizerConfig cauchy_config;
  cauchy_config.robust_kernel = RobustKernel::kCauchy;
  cauchy_config.robust_kernel_k = 1.0;
  cauchy_config.robustify_loop_constraints_only = true;
  cauchy_config.loop_constraint_min_id_separation = 5U;
  const auto cauchy = GtsamPoseGraphOptimizer(cauchy_config).optimize(initial, constraints);

  const double gaussian_error = std::abs(gaussian.poses.at(10).x - 10.0);
  const double cauchy_error = std::abs(cauchy.poses.at(10).x - 10.0);
  EXPECT_GT(gaussian_error, 5.0);
  EXPECT_LT(cauchy_error, 0.1);
  EXPECT_LT(cauchy_error, gaussian_error);
  EXPECT_EQ(cauchy.constraints_used, constraints.size());
  EXPECT_EQ(cauchy.robustified_constraints, 1U);
}

}  // namespace embodied_slam
