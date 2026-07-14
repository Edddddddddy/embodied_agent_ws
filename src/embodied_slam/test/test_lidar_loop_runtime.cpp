#include <cmath>
#include <stdexcept>
#include <vector>

#include "embodied_slam/lidar_loop_runtime.hpp"
#include "gtest/gtest.h"

namespace
{

std::vector<embodied_slam::LidarPoint2D> scan()
{
  std::vector<embodied_slam::LidarPoint2D> points;
  for (int index = 0; index < 40; ++index) {
    const double angle = 0.11 * static_cast<double>(index);
    const double radius = 1.0 + 0.07 * static_cast<double>(index % 11);
    points.push_back({radius * std::cos(angle), radius * std::sin(angle)});
  }
  return points;
}

embodied_slam::LiveLidarLoopDetectorConfig config()
{
  return {{8, 24, 10.0, 20}, {1.0, 10, 0.5}, 3, 0.5};
}

}  // namespace

TEST(LiveLidarLoopDetector, SamplesAndNeverMatchesCurrentScan)
{
  embodied_slam::LiveLidarLoopDetector detector(config());

  const auto first = detector.ingest(0.0, scan());
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->query_id, 0);
  EXPECT_EQ(first->indexed_scans, 1U);
  EXPECT_TRUE(first->candidates.empty());

  EXPECT_FALSE(detector.ingest(0.2, scan()).has_value());
  const auto second = detector.ingest(1.1, scan());
  ASSERT_TRUE(second.has_value());
  ASSERT_EQ(second->candidates.size(), 1U);
  EXPECT_EQ(second->candidates.front().scan_id, 0);
  EXPECT_NEAR(second->candidates.front().similarity, 1.0, 1e-12);
  EXPECT_EQ(detector.indexedScans(), 2U);
}

TEST(LiveLidarLoopDetector, IgnoresInvalidGeometryWithoutConsumingId)
{
  embodied_slam::LiveLidarLoopDetector detector(config());
  EXPECT_FALSE(detector.ingest(0.0, {{1.0, 0.0}}).has_value());

  const auto first = detector.ingest(0.6, scan());
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->query_id, 0);
}

TEST(LiveLidarLoopDetector, RejectsTimeRegressionAndInvalidConfig)
{
  embodied_slam::LiveLidarLoopDetector detector(config());
  ASSERT_TRUE(detector.ingest(2.0, scan()).has_value());
  EXPECT_THROW(detector.ingest(1.0, scan()), std::invalid_argument);

  auto invalid = config();
  invalid.top_k = 0;
  EXPECT_THROW(
    {
      const embodied_slam::LiveLidarLoopDetector invalid_detector{invalid};
      (void)invalid_detector;
    },
    std::invalid_argument);
}
