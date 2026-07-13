#include <gtest/gtest.h>

#include "embodied_navigation/constant_velocity_predictor.hpp"

namespace embodied_navigation
{

TEST(ConstantVelocityPredictor, PredictsPositionAndGrowingUncertainty)
{
  PredictionConfig config;
  config.horizon_s = 1.0;
  config.step_s = 0.5;
  config.radius_padding_m = 0.1;
  config.future_radius_growth_mps = 0.2;
  const auto points = predict_constant_velocity(
    {ObstacleMotion{1.0, 2.0, 0.5, -0.2, 0.25, 0.9}}, config);

  ASSERT_EQ(points.size(), 3U);
  EXPECT_NEAR(points.front().x, 1.0, 1e-9);
  EXPECT_NEAR(points.back().x, 1.5, 1e-9);
  EXPECT_NEAR(points.back().y, 1.8, 1e-9);
  EXPECT_NEAR(points.front().radius, 0.35, 1e-9);
  EXPECT_NEAR(points.back().radius, 0.55, 1e-9);
  EXPECT_NEAR(points.back().time_s, 1.0, 1e-9);
}

TEST(ConstantVelocityPredictor, RejectsLowConfidenceTracks)
{
  PredictionConfig config;
  config.minimum_confidence = 0.6;
  const auto points = predict_constant_velocity(
    {ObstacleMotion{0.0, 0.0, 1.0, 0.0, 0.2, 0.4}}, config);
  EXPECT_TRUE(points.empty());
}

TEST(ConstantVelocityPredictor, ClampsInvalidStepAndNegativeHorizon)
{
  PredictionConfig config;
  config.horizon_s = -1.0;
  config.step_s = 0.0;
  const auto points = predict_constant_velocity(
    {ObstacleMotion{0.0, 0.0, 1.0, 0.0, 0.2, 1.0}}, config);
  ASSERT_EQ(points.size(), 1U);
  EXPECT_NEAR(points.front().time_s, 0.0, 1e-9);
}

}  // namespace embodied_navigation
