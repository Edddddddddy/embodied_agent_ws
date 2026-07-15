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
  config.matcher.point_stride = 1U;
  config.matcher.minimum_points = 20U;
  config.matcher.minimum_inlier_ratio = 0.50;
  config.matcher.minimum_observability_ratio = 0.001;
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

}  // namespace embodied_slam
