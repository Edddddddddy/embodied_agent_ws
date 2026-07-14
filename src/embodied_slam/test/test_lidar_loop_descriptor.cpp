#include <cmath>
#include <stdexcept>
#include <vector>

#include "embodied_slam/lidar_loop_descriptor.hpp"
#include "gtest/gtest.h"

namespace
{

constexpr double kPi = 3.14159265358979323846;

std::vector<embodied_slam::LidarPoint2D> makeAsymmetricScan(const double yaw = 0.0)
{
  std::vector<embodied_slam::LidarPoint2D> points;
  for (int sector = 0; sector < 12; ++sector) {
    // 合成点放在离分箱边界足够远的位置；测试目标是循环移位语义，不是浮点边界取整。
    const double angle = yaw + 0.03 + static_cast<double>(sector) * kPi / 6.0;
    const int repeats = 2 + sector % 4;
    for (int ring = 0; ring < repeats; ++ring) {
      const double radius = 1.13 + 0.53 * static_cast<double>((sector * 3 + ring) % 8);
      points.push_back({radius * std::cos(angle), radius * std::sin(angle)});
    }
  }
  return points;
}

embodied_slam::PolarDescriptorConfig config()
{
  return {10, 24, 8.0, 20};
}

}  // namespace

TEST(LidarLoopDescriptor, RecoversCircularYawShift)
{
  const auto reference = embodied_slam::makePolarScanDescriptor(makeAsymmetricScan(), config());
  const auto rotated = embodied_slam::makePolarScanDescriptor(
    makeAsymmetricScan(3.0 * kPi / 12.0), config());

  const auto match = embodied_slam::alignPolarDescriptors(reference, rotated);

  EXPECT_NEAR(match.similarity, 1.0, 1e-12);
  EXPECT_EQ(match.sector_shift, 3U);
  EXPECT_NEAR(match.yaw_offset_rad, 3.0 * kPi / 12.0, 1e-12);
}

TEST(LidarLoopDescriptor, RejectsTooFewValidPoints)
{
  std::vector<embodied_slam::LidarPoint2D> points{{1.0, 0.0}, {2.0, 0.0}};
  const auto descriptor = embodied_slam::makePolarScanDescriptor(points, config());

  EXPECT_FALSE(descriptor.valid());
  EXPECT_THROW(
    embodied_slam::alignPolarDescriptors(descriptor, descriptor), std::invalid_argument);
}

TEST(LidarLoopCandidateIndex, ExcludesRecentHistoryAndRanksGeometry)
{
  const auto first = embodied_slam::makePolarScanDescriptor(makeAsymmetricScan(), config());
  auto changed_points = makeAsymmetricScan();
  for (auto & point : changed_points) {
    point.x *= 1.7;
    point.y *= 1.7;
  }
  const auto second = embodied_slam::makePolarScanDescriptor(changed_points, config());
  embodied_slam::LidarLoopCandidateIndex index({60.0, 10, 0.0});
  index.add(10, 0.0, first);
  index.add(20, 80.0, second);

  const auto early = index.query(100.0, first, 3);
  ASSERT_EQ(early.size(), 1U);
  EXPECT_EQ(early.front().scan_id, 10);

  const auto later = index.query(160.0, first, 2);
  ASSERT_EQ(later.size(), 2U);
  EXPECT_EQ(later.front().scan_id, 10);
  EXPECT_GT(later.front().similarity, later.back().similarity);
}

TEST(LidarLoopCandidateIndex, RejectsDuplicateIdsAndInvalidQueries)
{
  const auto descriptor = embodied_slam::makePolarScanDescriptor(makeAsymmetricScan(), config());
  embodied_slam::LidarLoopCandidateIndex index;
  index.add(1, 0.0, descriptor);

  EXPECT_THROW(index.add(1, 1.0, descriptor), std::invalid_argument);
  EXPECT_THROW(index.query(100.0, descriptor, 0), std::invalid_argument);
}
