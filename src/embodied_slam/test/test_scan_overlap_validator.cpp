#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "embodied_slam/scan_overlap_validator.hpp"

namespace embodied_slam
{
namespace
{

std::vector<ScanPoint2d> lineScan(const double offset_x = 0.0)
{
  std::vector<ScanPoint2d> points;
  for (int index = 0; index < 100; ++index) {
    points.push_back({offset_x + 0.02 * index, 0.2 * std::sin(0.1 * index)});
  }
  return points;
}

}  // namespace

TEST(ScanOverlapValidator, IdenticalScansHaveCompleteBidirectionalOverlap)
{
  ScanOverlapConfig config;
  config.point_stride = 1U;
  config.minimum_points = 10U;
  const auto scan = lineScan();
  const auto result = evaluateScanOverlap(scan, scan, {}, config);

  ASSERT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.overlap_ratio, 1.0);
  EXPECT_EQ(result.source_matches, scan.size());
  EXPECT_EQ(result.target_matches, scan.size());
}

TEST(ScanOverlapValidator, AppliesTheMeasuredRelativePoseBeforeScoring)
{
  ScanOverlapConfig config;
  config.match_distance_m = 0.03;
  config.point_stride = 1U;
  config.minimum_points = 10U;
  const auto source = lineScan();
  const auto target = lineScan(-1.0);
  const auto result = evaluateScanOverlap(source, target, {1.0, 0.0, 0.0}, config);

  ASSERT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.overlap_ratio, 1.0);
}

TEST(ScanOverlapValidator, DisjointScansHaveNoOverlap)
{
  ScanOverlapConfig config;
  config.match_distance_m = 0.05;
  config.point_stride = 1U;
  config.minimum_points = 10U;
  const auto result = evaluateScanOverlap(lineScan(), lineScan(10.0), {}, config);

  ASSERT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.overlap_ratio, 0.0);
}

TEST(ScanOverlapValidator, InsufficientOrInvalidPointsFailOpen)
{
  ScanOverlapConfig config;
  config.point_stride = 1U;
  config.minimum_points = 5U;
  const std::vector<ScanPoint2d> sparse = {
    {0.0, 0.0}, {1.0, 0.0}, {std::numeric_limits<double>::infinity(), 0.0}};
  const auto result = evaluateScanOverlap(sparse, sparse, {}, config);

  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.source_points, 2U);
  EXPECT_EQ(result.target_points, 2U);
}

TEST(ScanOverlapValidator, RejectsInvalidConfiguration)
{
  ScanOverlapConfig config;
  config.match_distance_m = 0.0;
  EXPECT_THROW(evaluateScanOverlap(lineScan(), lineScan(), {}, config), std::invalid_argument);
}

}  // namespace embodied_slam
