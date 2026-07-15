#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "embodied_slam/lidar_loop_verifier.hpp"

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

LiveLidarLoopVerifierConfig testConfig(const std::size_t cache_size = 4U)
{
  LiveLidarLoopVerifierConfig config;
  config.maximum_cached_scans = cache_size;
  config.maximum_cached_odometry = cache_size;
  config.matching_mode = LiveLidarMatchingMode::ScanToScan;
  config.matcher.point_stride = 1U;
  config.matcher.minimum_points = 20U;
  config.matcher.minimum_inlier_ratio = 0.50;
  config.matcher.minimum_observability_ratio = 0.001;
  return config;
}

LiveLidarLoopVerifierConfig submapTestConfig()
{
  auto config = testConfig(6U);
  config.matching_mode = LiveLidarMatchingMode::ScanToSubmap;
  config.maximum_odometry_time_delta_ns = 20;
  config.submap.half_window_scans = 1U;
  config.submap.maximum_time_delta_s = 1.0;
  config.submap.point_stride = 1U;
  config.submap.minimum_contributing_scans = 2U;
  config.submap.minimum_points = 40U;
  return config;
}

LiveLidarLoopCandidateInput candidate(const std::int64_t stamp_ns)
{
  return LiveLidarLoopCandidateInput{7, stamp_ns, 1U, 0.92, 0.0};
}

}  // namespace

TEST(LiveLidarLoopVerifier, VerifiesCachedGeometryAndPreservesCorrelation)
{
  LiveLidarLoopVerifier verifier(testConfig());
  verifier.cacheScan(100, structuredScan());
  verifier.cacheScan(200, structuredScan());

  const auto results = verifier.verify(200, {candidate(100)});

  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results.front().candidate.candidate_id, 7);
  EXPECT_EQ(results.front().candidate.rank, 1U);
  EXPECT_TRUE(results.front().match.available);
  EXPECT_TRUE(results.front().match.accepted) << results.front().match.rejection_reason;
  EXPECT_GT(results.front().match.bidirectional_overlap_ratio, 0.95);
}

TEST(LiveLidarLoopVerifier, ReportsMissingQueryAndEvictedCandidate)
{
  LiveLidarLoopVerifier verifier(testConfig(2U));
  verifier.cacheScan(100, structuredScan());

  const auto missing_query = verifier.verify(200, {candidate(100)});
  ASSERT_EQ(missing_query.size(), 1U);
  EXPECT_EQ(missing_query.front().match.rejection_reason, "query_scan_not_cached");

  verifier.cacheScan(200, structuredScan());
  verifier.cacheScan(300, structuredScan());
  EXPECT_EQ(verifier.cachedScans(), 2U);
  EXPECT_FALSE(verifier.hasScan(100));
  const auto evicted_candidate = verifier.verify(300, {candidate(100)});
  ASSERT_EQ(evicted_candidate.size(), 1U);
  EXPECT_EQ(evicted_candidate.front().match.rejection_reason, "candidate_scan_not_cached");
}

TEST(LiveLidarLoopVerifier, UpdatesDuplicateTimestampWithoutBreakingEvictionOrder)
{
  LiveLidarLoopVerifier verifier(testConfig(2U));
  verifier.cacheScan(100, structuredScan());
  verifier.cacheScan(100, structuredScan());
  verifier.cacheScan(200, structuredScan());
  EXPECT_EQ(verifier.cachedScans(), 2U);
  EXPECT_TRUE(verifier.hasScan(100));

  auto invalid = structuredScan();
  invalid.front().x = std::numeric_limits<double>::infinity();
  EXPECT_THROW(verifier.cacheScan(300, std::move(invalid)), std::invalid_argument);
}

TEST(LiveLidarLoopVerifier, BuildsShortOdometrySubmapsBeforeMatching)
{
  LiveLidarLoopVerifier verifier(submapTestConfig());
  for (const std::int64_t stamp : {100, 200, 300}) {
    verifier.cacheOdometry(stamp + 5, {0.0, 0.0, 0.0});
    verifier.cacheScan(stamp, structuredScan());
  }

  const auto results = verifier.verify(300, {candidate(100)});

  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(verifier.matchingMode(), LiveLidarMatchingMode::ScanToSubmap);
  EXPECT_EQ(results.front().query_contributing_scans, 2U);
  EXPECT_EQ(results.front().candidate_contributing_scans, 2U);
  EXPECT_GE(results.front().match.source_points, 160U);
  EXPECT_GE(results.front().match.target_points, 160U);
  EXPECT_TRUE(results.front().match.accepted) << results.front().match.rejection_reason;
}

TEST(LiveLidarLoopVerifier, RejectsSubmapWhenOdometryAssociationIsStale)
{
  auto config = submapTestConfig();
  config.maximum_odometry_time_delta_ns = 10;
  LiveLidarLoopVerifier verifier(config);
  verifier.cacheScan(100, structuredScan());
  verifier.cacheScan(200, structuredScan());
  verifier.cacheOdometry(1000, {0.0, 0.0, 0.0});

  EXPECT_FALSE(verifier.hasGeometry(200));
  const auto results = verifier.verify(200, {candidate(100)});
  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(
    results.front().match.rejection_reason,
    "query_submap_missing_center_scan_or_odometry");
}

TEST(LiveLidarLoopVerifier, ValidatesMatchingModeAndOdometryInput)
{
  EXPECT_EQ(
    liveLidarMatchingModeFromString("scan_to_scan"), LiveLidarMatchingMode::ScanToScan);
  EXPECT_EQ(toString(LiveLidarMatchingMode::ScanToSubmap), "scan_to_submap");
  EXPECT_THROW(liveLidarMatchingModeFromString("unknown"), std::invalid_argument);

  LiveLidarLoopVerifier verifier(submapTestConfig());
  EXPECT_THROW(
    verifier.cacheOdometry(100, {std::numeric_limits<double>::infinity(), 0.0, 0.0}),
    std::invalid_argument);
}

}  // namespace embodied_slam
