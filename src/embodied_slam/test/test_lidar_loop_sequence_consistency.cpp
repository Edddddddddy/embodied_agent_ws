#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "embodied_slam/lidar_loop_sequence_consistency.hpp"

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

TEST(LidarLoopSequenceConsistency, PreservesCoherentSecondRankHypothesis)
{
  LidarLoopSequenceConsistencyConfig config;
  config.minimum_confirmations = 3U;
  config.maximum_pair_age_delta_s = 0.25;
  config.maximum_translation_delta_m = 0.35;
  config.maximum_yaw_delta_rad = 0.20;
  config.maximum_hypotheses = 16U;
  LidarLoopSequenceConsistency consistency(config);

  const auto first = consistency.observeBatch({
        observation(100, 40, 100.0, 40.0, {2.0, 0.0, 0.0}),
        observation(100, 1, 100.0, 20.0, {0.8, 0.0, 0.0})});
  const auto second = consistency.observeBatch({
        observation(101, 42, 100.5, 39.0, {-2.0, 0.0, 1.0}),
        observation(101, 2, 100.5, 20.5, {0.9, 0.0, 0.05})});
  const auto third = consistency.observeBatch({
        observation(102, 44, 101.0, 45.0, {2.5, 0.0, -1.0}),
        observation(102, 3, 101.0, 21.0, {1.0, 0.0, 0.08})});

  EXPECT_EQ(first[1].confirmation_count, 1U);
  EXPECT_EQ(second[1].confirmation_count, 2U);
  EXPECT_FALSE(second[1].approved);
  EXPECT_TRUE(third[1].approved);
  EXPECT_EQ(third[1].confirmation_count, 3U);
  EXPECT_EQ(third[1].reason, "sequence_consistency_confirmed");
  EXPECT_FALSE(third[0].approved);
}

TEST(LidarLoopSequenceConsistency, NonMonotonicBatchDoesNotMutateTracks)
{
  LidarLoopSequenceConsistencyConfig config;
  config.minimum_confirmations = 2U;
  LidarLoopSequenceConsistency consistency(config);
  (void)consistency.observeBatch({
        observation(100, 1, 100.0, 20.0, {1.0, 0.0, 0.0})});
  const auto late = consistency.observeBatch({
        observation(99, 2, 99.0, 19.0, {1.0, 0.0, 0.0})});
  EXPECT_EQ(late[0].reason, "sequence_non_monotonic_query");
  const auto next = consistency.observeBatch({
        observation(101, 2, 100.5, 20.5, {1.1, 0.0, 0.05})});
  EXPECT_TRUE(next[0].approved);
  EXPECT_EQ(next[0].confirmation_count, 2U);
}

TEST(LidarLoopSequenceConsistency, RejectsMixedQueryBatchAndInvalidConfig)
{
  LidarLoopSequenceConsistency consistency;
  EXPECT_THROW(
    (void)consistency.observeBatch({
        observation(100, 1, 100.0, 20.0, {1.0, 0.0, 0.0}),
        observation(101, 2, 100.5, 20.5, {1.1, 0.0, 0.0})}),
    std::invalid_argument);
  EXPECT_THROW(
    []() {
      LidarLoopSequenceConsistencyConfig config;
      config.maximum_hypotheses = 0U;
      (void)LidarLoopSequenceConsistency(config);
    }(),
    std::invalid_argument);
}

}  // namespace
}  // namespace embodied_slam
