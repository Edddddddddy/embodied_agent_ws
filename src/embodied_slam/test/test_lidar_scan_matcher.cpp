#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "embodied_slam/lidar_scan_matcher.hpp"

namespace embodied_slam
{
namespace
{

std::vector<LidarPoint2D> structuredScan()
{
  std::vector<LidarPoint2D> points;
  for (int index = 0; index < 80; ++index) {
    const double angle = 0.073 * static_cast<double>(index);
    const double radius = 2.0 + 0.5 * std::sin(3.0 * angle) + 0.2 * std::cos(7.0 * angle);
    points.push_back({radius * std::cos(angle), radius * std::sin(angle)});
  }
  return points;
}

std::vector<LidarPoint2D> inverseTransform(
  const std::vector<LidarPoint2D> & source, const Pose2d & target_to_source)
{
  const double cosine = std::cos(target_to_source.yaw);
  const double sine = std::sin(target_to_source.yaw);
  std::vector<LidarPoint2D> target;
  target.reserve(source.size());
  for (const auto & point : source) {
    const double shifted_x = point.x - target_to_source.x;
    const double shifted_y = point.y - target_to_source.y;
    target.push_back({
      cosine * shifted_x + sine * shifted_y,
      -sine * shifted_x + cosine * shifted_y});
  }
  return target;
}

LidarScanMatcherConfig testConfig()
{
  LidarScanMatcherConfig config;
  config.point_stride = 1U;
  config.minimum_points = 20U;
  config.minimum_inlier_ratio = 0.50;
  config.minimum_observability_ratio = 0.001;
  return config;
}

}  // namespace

TEST(LidarScanMatcher, RecoversRigidTransformFromDescriptorYawSeed)
{
  const auto source = structuredScan();
  const Pose2d expected{0.35, -0.22, 0.31};
  const auto target = inverseTransform(source, expected);

  const auto result = matchLidarScans(source, target, -expected.yaw, testConfig());

  ASSERT_TRUE(result.available);
  ASSERT_TRUE(result.accepted) << result.rejection_reason;
  EXPECT_NEAR(result.target_to_source.x, expected.x, 0.02);
  EXPECT_NEAR(result.target_to_source.y, expected.y, 0.02);
  EXPECT_NEAR(result.target_to_source.yaw, expected.yaw, 0.01);
  EXPECT_GT(result.bidirectional_overlap_ratio, 0.95);
  EXPECT_LT(result.rmse_m, 0.02);
}

TEST(LidarScanMatcher, RejectsSparseAndInvalidConfiguration)
{
  auto config = testConfig();
  const std::vector<LidarPoint2D> sparse{{0.0, 0.0}, {1.0, 0.0}};
  const auto result = matchLidarScans(sparse, sparse, 0.0, config);
  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.rejection_reason, "insufficient_points");

  config.fine_max_correspondence_m = config.coarse_max_correspondence_m;
  EXPECT_THROW(matchLidarScans(structuredScan(), structuredScan(), 0.0, config), std::invalid_argument);
  EXPECT_THROW(
    matchLidarScans(
      structuredScan(), structuredScan(), std::numeric_limits<double>::infinity(), testConfig()),
    std::invalid_argument);
}

TEST(LidarScanMatcher, RejectsTranslationOutsideSafetyEnvelope)
{
  const auto source = structuredScan();
  const Pose2d expected{1.2, 0.0, 0.1};
  const auto target = inverseTransform(source, expected);
  auto config = testConfig();
  config.maximum_translation_m = 0.5;

  const auto result = matchLidarScans(source, target, expected.yaw, config);

  ASSERT_TRUE(result.available);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.rejection_reason, "translation_limit");
}

TEST(LidarScanMatcher, RejectsHalfTurnAmbiguityInSymmetricCorridor)
{
  std::vector<LidarPoint2D> corridor;
  for (int index = -30; index <= 30; ++index) {
    const double x = 0.1 * static_cast<double>(index);
    corridor.push_back({x, -1.0});
    corridor.push_back({x, 1.0});
  }
  auto config = testConfig();
  config.minimum_points = 40U;

  const auto result = matchLidarScans(corridor, corridor, 3.14159265358979323846, config);

  ASSERT_TRUE(result.available);
  EXPECT_TRUE(result.yaw_ambiguous);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.rejection_reason, "yaw_ambiguous");
  EXPECT_GT(result.alternate_yaw_separation_rad, 3.0);
}

TEST(LidarScanMatcher, OdometryPriorResolvesSymmetricCorridorWithoutGroundTruth)
{
  std::vector<LidarPoint2D> corridor;
  for (int index = -30; index <= 30; ++index) {
    const double x = 0.1 * static_cast<double>(index);
    corridor.push_back({x, -1.0});
    corridor.push_back({x, 1.0});
  }
  auto config = testConfig();
  config.minimum_points = 40U;

  const auto result = matchLidarScans(
    corridor, corridor, 3.14159265358979323846, config, Pose2d{});

  ASSERT_TRUE(result.available);
  EXPECT_TRUE(result.odometry_prior_used);
  EXPECT_TRUE(result.odometry_prior_consistent);
  EXPECT_FALSE(result.yaw_ambiguous);
  EXPECT_TRUE(result.accepted) << result.rejection_reason;
  EXPECT_NEAR(result.target_to_source.yaw, 0.0, 1e-6);
}

TEST(LidarScanMatcher, LongRangeOdometryTranslationDoesNotVetoYawDisambiguation)
{
  std::vector<LidarPoint2D> corridor;
  for (int index = -30; index <= 30; ++index) {
    const double x = 0.1 * static_cast<double>(index);
    corridor.push_back({x, -1.0});
    corridor.push_back({x, 1.0});
  }
  auto config = testConfig();
  config.minimum_points = 40U;

  // 回环间隔很长时平移里程计可能已漂移数米，但其偏航仍能排除 180 度镜像解。
  const auto result = matchLidarScans(
    corridor, corridor, 3.14159265358979323846, config, Pose2d{10.0, -4.0, 0.0});

  ASSERT_TRUE(result.available);
  EXPECT_TRUE(result.odometry_prior_consistent);
  EXPECT_TRUE(result.accepted) << result.rejection_reason;
  EXPECT_NEAR(result.target_to_source.yaw, 0.0, 1e-6);
  EXPECT_GT(result.odometry_prior_translation_error_m, 10.0);
}

}  // namespace embodied_slam
