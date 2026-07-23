#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <vector>

#include "embodied_navigation/gated_observation_assignment.hpp"

namespace embodied_navigation
{

TEST(GatedObservationAssignment, GlobalSolutionAvoidsGreedyTrackFragmentation)
{
  const std::vector<Point2d> predictions{{0.0, 0.0}, {0.3, 0.0}};
  // 第一条轨迹对两个观测等距，贪心会先占用 observation[0]，使第二条轨迹
  // 唯一合理的观测丢失；全局解会联合考虑两条边。
  const std::vector<Point2d> observations{{0.2, 0.0}, {-0.2, 0.0}};

  const auto greedy = assign_gated_observations(
    predictions, observations, 0.5, AssociationStrategy::GreedyNearest);
  const auto global = assign_gated_observations(
    predictions, observations, 0.5, AssociationStrategy::GlobalNearest);

  ASSERT_EQ(greedy.size(), 2U);
  EXPECT_EQ(greedy[0], 0U);
  EXPECT_FALSE(greedy[1].has_value());
  ASSERT_EQ(global.size(), 2U);
  EXPECT_EQ(global[0], 1U);
  EXPECT_EQ(global[1], 0U);
}

TEST(GatedObservationAssignment, LeavesGatedTracksUnmatched)
{
  const auto assignment = assign_gated_observations(
    {{0.0, 0.0}, {10.0, 0.0}}, {{0.1, 0.0}}, 0.5,
    AssociationStrategy::GlobalNearest);

  ASSERT_EQ(assignment.size(), 2U);
  EXPECT_EQ(assignment[0], 0U);
  EXPECT_FALSE(assignment[1].has_value());
}

TEST(GatedObservationAssignment, MahalanobisCostUsesPerTrackUncertainty)
{
  const std::vector<AssociationPrediction> predictions{
    {{0.0, 0.0}, {0.0025, 0.0025}},
    {{0.4, 0.0}, {0.25, 0.25}}};
  // track_1 很确定，应匹配 0.1；track_2 遮挡后不确定，可接受跨过预测均值的 -0.2。
  const std::vector<Point2d> observations{{-0.2, 0.0}, {0.1, 0.0}};

  const auto euclidean = assign_gated_observations(
    predictions, observations, 0.0025, 1.0, 9.210,
    AssociationStrategy::GlobalNearest, AssociationMetric::Euclidean);
  const auto mahalanobis = assign_gated_observations(
    predictions, observations, 0.0025, 1.0, 9.210,
    AssociationStrategy::GlobalNearest, AssociationMetric::Mahalanobis);

  ASSERT_EQ(euclidean.size(), 2U);
  EXPECT_EQ(euclidean[0], 0U);
  EXPECT_EQ(euclidean[1], 1U);
  ASSERT_EQ(mahalanobis.size(), 2U);
  EXPECT_EQ(mahalanobis[0], 1U);
  EXPECT_EQ(mahalanobis[1], 0U);
}

TEST(GatedObservationAssignment, MahalanobisStillAppliesHardDistanceGate)
{
  const auto assignment = assign_gated_observations(
    {{{0.0, 0.0}, {100.0, 100.0}}}, {{2.0, 0.0}}, 0.01, 0.8, 9.210,
    AssociationStrategy::GlobalNearest, AssociationMetric::Mahalanobis);
  ASSERT_EQ(assignment.size(), 1U);
  EXPECT_FALSE(assignment[0].has_value());
}

TEST(GatedObservationAssignment, ValidatesStrategyAndFiniteInputs)
{
  EXPECT_EQ(
    association_strategy_from_string("global_nearest"), AssociationStrategy::GlobalNearest);
  EXPECT_EQ(association_strategy_from_string("greedy"), AssociationStrategy::GreedyNearest);
  EXPECT_EQ(association_metric_from_string("mahalanobis"), AssociationMetric::Mahalanobis);
  EXPECT_EQ(association_metric_from_string("euclidean"), AssociationMetric::Euclidean);
  EXPECT_THROW(association_strategy_from_string("magic"), std::invalid_argument);
  EXPECT_THROW(association_metric_from_string("magic"), std::invalid_argument);
  EXPECT_THROW(
    assign_gated_observations({}, {}, 0.0, AssociationStrategy::GlobalNearest),
    std::invalid_argument);
  EXPECT_THROW(
    assign_gated_observations(
      {{std::numeric_limits<double>::infinity(), 0.0}}, {}, 1.0,
      AssociationStrategy::GlobalNearest),
    std::invalid_argument);
  EXPECT_THROW(
    assign_gated_observations(
      {{{0.0, 0.0}, {-1.0, 0.1}}}, {}, 0.01, 1.0, 9.210,
      AssociationStrategy::GlobalNearest, AssociationMetric::Mahalanobis),
    std::invalid_argument);
}

}  // namespace embodied_navigation
