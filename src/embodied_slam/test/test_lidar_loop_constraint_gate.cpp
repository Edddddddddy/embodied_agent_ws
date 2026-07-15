#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "embodied_slam/lidar_loop_constraint_gate.hpp"

namespace embodied_slam
{
namespace
{

LidarLoopConstraintInput eligible(
  const std::int64_t query_id = 20, const std::int64_t candidate_id = 1,
  const std::int64_t query_stamp_ns = 2000000000LL,
  const std::int64_t candidate_stamp_ns = 1000000000LL)
{
  LidarLoopConstraintInput input;
  input.query_id = query_id;
  input.query_stamp_ns = query_stamp_ns;
  input.candidate_id = candidate_id;
  input.candidate_stamp_ns = candidate_stamp_ns;
  input.rank = 1U;
  input.matching_mode = "scan_to_submap";
  input.available = true;
  input.converged = true;
  input.accepted = true;
  input.target_to_source = {-1.0, 0.0, 0.0};
  input.query_submap_scans = 3U;
  input.candidate_submap_scans = 3U;
  input.correspondences = 80U;
  input.descriptor_similarity = 0.90;
  input.inlier_ratio = 0.80;
  input.overlap_ratio = 0.75;
  input.rmse_m = 0.04;
  input.observability_ratio = 0.20;
  return input;
}

TEST(LidarLoopConstraintGate, ShadowModeSelectsOneAuditableCandidate)
{
  auto weaker = eligible(20, 2, 2000000000LL, 900000000LL);
  weaker.rank = 2U;
  weaker.overlap_ratio = 0.60;
  const auto decisions = LidarLoopConstraintGate().evaluate({weaker, eligible()});
  ASSERT_EQ(decisions.size(), 2U);
  EXPECT_FALSE(decisions[0].policy_approved);
  EXPECT_EQ(decisions[0].reason, "lower_quality_candidate");
  EXPECT_TRUE(decisions[1].policy_approved);
  EXPECT_FALSE(decisions[1].commit_requested);
  EXPECT_EQ(decisions[1].reason, "shadow_mode");
  EXPECT_GT(decisions[1].quality_score, decisions[0].quality_score);
  EXPECT_DOUBLE_EQ(decisions[1].covariance[0], 0.04);
  EXPECT_DOUBLE_EQ(decisions[1].covariance[8], 0.04);
}

TEST(LidarLoopConstraintGate, CommitModeDeduplicatesAndRateLimits)
{
  LidarLoopConstraintGateConfig config;
  config.commit_enabled = true;
  config.minimum_commit_query_separation = 5;
  LidarLoopConstraintGate gate(config);

  auto first = gate.evaluate({eligible(20, 1, 2000000000LL, 1000000000LL)});
  ASSERT_EQ(first.size(), 1U);
  EXPECT_TRUE(first[0].policy_approved);
  EXPECT_TRUE(first[0].commit_requested);
  EXPECT_EQ(first[0].reason, "commit_requested");

  const auto duplicate = gate.evaluate({eligible(20, 1, 2000000000LL, 1000000000LL)});
  EXPECT_EQ(duplicate[0].reason, "duplicate_pair");

  const auto limited = gate.evaluate({eligible(23, 2, 3000000000LL, 1200000000LL)});
  EXPECT_FALSE(limited[0].policy_approved);
  EXPECT_FALSE(limited[0].commit_requested);
  EXPECT_EQ(limited[0].reason, "commit_rate_limited");

  const auto next = gate.evaluate({eligible(25, 3, 4000000000LL, 1300000000LL)});
  EXPECT_TRUE(next[0].commit_requested);
}

TEST(LidarLoopConstraintGate, RejectsWeakAmbiguousAndScanToScanInputs)
{
  LidarLoopConstraintGate gate;
  auto weak = eligible();
  weak.overlap_ratio = 0.20;
  auto ambiguous = eligible(21, 2, 3000000000LL, 1000000000LL);
  ambiguous.yaw_ambiguous = true;
  auto scan_to_scan = eligible(22, 3, 4000000000LL, 1000000000LL);
  scan_to_scan.matching_mode = "scan_to_scan";
  const auto decisions = gate.evaluate({weak, ambiguous, scan_to_scan});
  EXPECT_EQ(decisions[0].reason, "low_bidirectional_overlap");
  EXPECT_EQ(decisions[1].reason, "yaw_ambiguous");
  EXPECT_EQ(decisions[2].reason, "matching_mode_not_allowed");
}

TEST(LidarLoopConstraintGate, ResetClearsHistorySequenceAndRateLimit)
{
  LidarLoopConstraintGateConfig config;
  config.commit_enabled = true;
  LidarLoopConstraintGate gate(config);
  EXPECT_TRUE(gate.evaluate({eligible()})[0].commit_requested);
  gate.reset();
  const auto replay = gate.evaluate({eligible()});
  EXPECT_EQ(replay[0].sequence, 1U);
  EXPECT_TRUE(replay[0].commit_requested);
}

TEST(LidarLoopConstraintGate, RejectsInvalidConfiguration)
{
  LidarLoopConstraintGateConfig config;
  config.maximum_history = 0U;
  EXPECT_THROW((void)LidarLoopConstraintGate(config), std::invalid_argument);
}

}  // namespace
}  // namespace embodied_slam
