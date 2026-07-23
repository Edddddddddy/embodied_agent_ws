#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "embodied_slam/loop_constraint_adapter.hpp"

namespace embodied_slam
{
namespace
{

TEST(LoopConstraintAdapter, ResolvesTimestampsAndInvertsIcpTransform)
{
  const std::vector<LoopConstraintScanRecord> scans = {
    {10, 1000000010LL, {2.0, 1.0, 0.0}, {2.0, 1.0, 0.0}},
    {11, 1500000000LL, {2.5, 1.0, 0.0}, {2.5, 1.0, 0.0}},
    {12, 2000000010LL, {3.0, 1.0, 0.0}, {3.0, 1.0, 0.0}}};
  const auto resolved = resolveLoopConstraint(
    {2000000000LL, 1000000000LL, {-1.0, 0.0, 0.0}}, scans, 25LL);
  ASSERT_TRUE(resolved.available);
  EXPECT_EQ(resolved.query.unique_id, 12);
  EXPECT_EQ(resolved.candidate.unique_id, 10);
  EXPECT_NEAR(resolved.query_sensor_pose.x, 3.0, 1e-9);
  EXPECT_NEAR(resolved.query_sensor_pose.y, 1.0, 1e-9);
  EXPECT_NEAR(resolved.query_sensor_pose.yaw, 0.0, 1e-9);
}

TEST(LoopConstraintAdapter, AppliesCandidateHeadingWhenComposingGlobalPose)
{
  constexpr double half_pi = 1.5707963267948966;
  const std::vector<LoopConstraintScanRecord> scans = {
    {1, 1000, {4.0, 2.0, half_pi}, {4.0, 2.0, half_pi}},
    {2, 2000, {4.0, 3.0, half_pi}, {4.0, 3.0, half_pi}}};
  const auto resolved = resolveLoopConstraint({2000, 1000, {-1.0, 0.0, 0.0}}, scans, 1);
  ASSERT_TRUE(resolved.available);
  EXPECT_NEAR(resolved.query_sensor_pose.x, 4.0, 1e-9);
  EXPECT_NEAR(resolved.query_sensor_pose.y, 3.0, 1e-9);
  EXPECT_NEAR(resolved.query_sensor_pose.yaw, half_pi, 1e-9);
}

TEST(LoopConstraintAdapter, RejectsMissingOrCollapsedTimestampAssociations)
{
  const std::vector<LoopConstraintScanRecord> scans = {
    {1, 1000, {}, {}}, {2, 2000, {}, {}}};
  EXPECT_EQ(
    resolveLoopConstraint({4000, 1000, {}}, scans, 10).reason,
    "query_scan_not_resolved");
  EXPECT_EQ(
    resolveLoopConstraint({2000, 1900, {}}, scans, 200).reason,
    "resolved_to_same_scan");
}

TEST(LoopConstraintAdapter, RejectsInvalidRequest)
{
  EXPECT_THROW(resolveLoopConstraint({1000, 2000, {}}, {}, 10), std::invalid_argument);
  EXPECT_THROW(resolveLoopConstraint({2000, 1000, {}}, {}, 0), std::invalid_argument);
}

}  // namespace
}  // namespace embodied_slam
