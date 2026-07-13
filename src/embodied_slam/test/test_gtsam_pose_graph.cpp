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

}  // namespace embodied_slam
