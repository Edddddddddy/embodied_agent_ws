#include <gtest/gtest.h>

#include <stdexcept>

#include "embodied_slam/lidar_loop_temporal_consistency.hpp"

namespace embodied_slam
{
namespace
{

LidarLoopTemporalObservation observation(
  const std::int64_t query_id, const std::int64_t candidate_id,
  const double query_stamp_s, const double candidate_stamp_s,
  const Pose2d & pose)
{
  return {
    query_id,
    static_cast<std::int64_t>(query_stamp_s * 1.0e9),
    candidate_id,
    static_cast<std::int64_t>(candidate_stamp_s * 1.0e9),
    pose};
}

TEST(LidarLoopTemporalConsistency, RequiresFourCoherentObservations)
{
  LidarLoopTemporalConsistency consistency;
  const auto first = consistency.observe(observation(100, 1, 100.0, 20.0, {1.0, 0.0, 0.0}));
  const auto second = consistency.observe(observation(101, 2, 100.5, 20.5, {1.1, 0.0, 0.05}));
  const auto third = consistency.observe(observation(102, 3, 101.0, 21.0, {1.2, 0.0, 0.08}));
  const auto fourth = consistency.observe(observation(103, 4, 101.5, 21.5, {1.3, 0.0, 0.10}));
  EXPECT_FALSE(first.approved);
  EXPECT_EQ(first.confirmation_count, 1U);
  EXPECT_FALSE(second.approved);
  EXPECT_EQ(second.confirmation_count, 2U);
  EXPECT_FALSE(third.approved);
  EXPECT_EQ(third.confirmation_count, 3U);
  EXPECT_TRUE(fourth.approved);
  EXPECT_EQ(fourth.confirmation_count, 4U);
  EXPECT_EQ(fourth.reason, "temporal_consistency_confirmed");
}

TEST(LidarLoopTemporalConsistency, ResetsTrackOnPairAgeOrTransformJump)
{
  LidarLoopTemporalConsistency consistency;
  (void)consistency.observe(observation(100, 1, 100.0, 20.0, {1.0, 0.0, 0.0}));
  const auto age_jump = consistency.observe(
    observation(101, 2, 100.5, 18.0, {1.1, 0.0, 0.05}));
  EXPECT_EQ(age_jump.confirmation_count, 1U);
  EXPECT_EQ(age_jump.reason, "temporal_pair_age_reset");
  const auto transform_jump = consistency.observe(
    observation(102, 3, 101.0, 18.5, {2.0, 0.0, 0.05}));
  EXPECT_EQ(transform_jump.confirmation_count, 1U);
  EXPECT_EQ(transform_jump.reason, "temporal_transform_reset");
}

TEST(LidarLoopTemporalConsistency, LateObservationDoesNotMutateTrack)
{
  LidarLoopTemporalConsistency consistency;
  (void)consistency.observe(observation(100, 1, 100.0, 20.0, {1.0, 0.0, 0.0}));
  const auto late = consistency.observe(observation(99, 2, 99.0, 19.0, {1.0, 0.0, 0.0}));
  EXPECT_EQ(late.reason, "temporal_non_monotonic_query");
  const auto next = consistency.observe(observation(101, 2, 100.5, 20.5, {1.1, 0.0, 0.05}));
  EXPECT_EQ(next.confirmation_count, 2U);
}

TEST(LidarLoopTemporalConsistency, OneConfirmationCanPreserveLegacyPolicy)
{
  LidarLoopTemporalConsistencyConfig config;
  config.minimum_confirmations = 1U;
  LidarLoopTemporalConsistency consistency(config);
  EXPECT_TRUE(consistency.observe(
    observation(100, 1, 100.0, 20.0, {1.0, 0.0, 0.0})).approved);
}

TEST(LidarLoopTemporalConsistency, RejectsInvalidConfiguration)
{
  LidarLoopTemporalConsistencyConfig config;
  config.maximum_query_gap_s = 0.0;
  EXPECT_THROW((void)LidarLoopTemporalConsistency(config), std::invalid_argument);
}

}  // namespace
}  // namespace embodied_slam
