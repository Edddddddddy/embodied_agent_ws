#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "embodied_slam/lidar_submap_builder.hpp"

namespace embodied_slam
{

TEST(LidarSubmapBuilder, TransformsNeighborScansIntoCenterFrame)
{
  const std::vector<LidarScanRecord> scans{
    {1, 0.0, {{0.0, 0.0}}},
    {2, 1.0, {{0.0, 0.0}}},
    {3, 2.0, {{0.0, 0.0}}}};
  const std::vector<LidarOdometryRecord> odometry{
    {1, 0.0, {0.0, 0.0, 0.0}},
    {2, 1.0, {1.0, 0.0, 0.0}},
    {3, 2.0, {2.0, 0.0, 0.0}}};
  LidarSubmapConfig config;
  config.maximum_time_delta_s = 1.1;
  config.point_stride = 1U;
  config.minimum_contributing_scans = 3U;
  config.minimum_points = 3U;

  const LidarSubmapBuilder builder(scans, odometry, config);
  const auto submap = builder.build(2);

  ASSERT_TRUE(submap.available) << submap.rejection_reason;
  EXPECT_EQ(submap.contributing_scan_ids, (std::vector<int>{1, 2, 3}));
  ASSERT_EQ(submap.points.size(), 3U);
  std::vector<double> x;
  for (const auto & point : submap.points) {x.push_back(point.x);}
  std::sort(x.begin(), x.end());
  EXPECT_NEAR(x[0], -1.0, 1e-9);
  EXPECT_NEAR(x[1], 0.0, 1e-9);
  EXPECT_NEAR(x[2], 1.0, 1e-9);
}

TEST(LidarSubmapBuilder, RejectsMissingCenterAndInsufficientWindow)
{
  const std::vector<LidarScanRecord> scans{
    {1, 0.0, {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}}},
    {2, 10.0, {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}}}};
  const std::vector<LidarOdometryRecord> odometry{
    {1, 0.0, {0.0, 0.0, 0.0}},
    {2, 10.0, {0.0, 0.0, 0.0}}};
  LidarSubmapConfig config;
  config.maximum_time_delta_s = 0.5;
  config.point_stride = 1U;
  config.minimum_points = 3U;
  const LidarSubmapBuilder builder(scans, odometry, config);

  EXPECT_EQ(builder.build(99).rejection_reason, "missing_center_scan_or_odometry");
  EXPECT_EQ(builder.build(1).rejection_reason, "insufficient_contributing_scans");
}

TEST(LidarSubmapBuilder, AppliesCenterFrameRotation)
{
  const double half_pi = 1.5707963267948966;
  const std::vector<LidarScanRecord> scans{
    {1, 0.0, {{0.0, 0.0}}},
    {2, 1.0, {{0.0, 0.0}}}};
  const std::vector<LidarOdometryRecord> odometry{
    {1, 0.0, {0.0, 0.0, half_pi}},
    {2, 1.0, {1.0, 0.0, half_pi}}};
  LidarSubmapConfig config;
  config.maximum_time_delta_s = 2.0;
  config.point_stride = 1U;
  config.minimum_contributing_scans = 2U;
  config.minimum_points = 2U + 1U;
  // 每帧补足点数，避免测试把“坐标变换”与 minimum_points 门限耦合。
  auto dense_scans = scans;
  dense_scans[0].points.push_back({0.0, 0.0});
  dense_scans[1].points.push_back({0.0, 0.0});

  const LidarSubmapBuilder builder(dense_scans, odometry, config);
  const auto submap = builder.build(1);

  ASSERT_TRUE(submap.available) << submap.rejection_reason;
  // 世界坐标 +x 在朝向 +90° 的中心帧中应落在 -y。
  EXPECT_NEAR(submap.points.back().x, 0.0, 1e-9);
  EXPECT_NEAR(submap.points.back().y, -1.0, 1e-9);
}

}  // namespace embodied_slam
