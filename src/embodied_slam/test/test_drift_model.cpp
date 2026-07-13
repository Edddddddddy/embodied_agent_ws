#include <gtest/gtest.h>

#include <vector>

#include "embodied_slam/drift_model.hpp"

namespace embodied_slam
{

TEST(DriftModel, IsDeterministicForAConfiguredSeed)
{
  DriftConfig config;
  config.random_seed = 17U;
  DriftModel first(config);
  DriftModel second(config);
  const std::vector<Pose2d> poses = {
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 1.57}, {1.0, 1.0, 1.57}};
  for (const auto & pose : poses) {
    const auto first_value = first.update(pose);
    const auto second_value = second.update(pose);
    EXPECT_DOUBLE_EQ(first_value.x, second_value.x);
    EXPECT_DOUBLE_EQ(first_value.y, second_value.y);
    EXPECT_DOUBLE_EQ(first_value.yaw, second_value.yaw);
  }
}

TEST(DriftModel, AccumulatesScaleAndHeadingBias)
{
  DriftConfig config;
  config.linear_scale = 1.10;
  config.yaw_bias_per_meter = 0.10;
  config.translation_noise_stddev = 0.0;
  config.yaw_noise_stddev = 0.0;
  DriftModel model(config);
  model.update(Pose2d{0.0, 0.0, 0.0});
  const Pose2d estimate = model.update(Pose2d{1.0, 0.0, 0.0});
  EXPECT_NEAR(estimate.x, 1.10, 1e-9);
  EXPECT_NEAR(estimate.y, 0.0, 1e-9);
  EXPECT_NEAR(estimate.yaw, 0.10, 1e-9);
}

TEST(DriftModel, ResetReplaysTheSameNoiseSequence)
{
  DriftModel model;
  model.update(Pose2d{0.0, 0.0, 0.0});
  const Pose2d first = model.update(Pose2d{1.0, 0.0, 0.0});
  model.reset();
  model.update(Pose2d{0.0, 0.0, 0.0});
  const Pose2d replay = model.update(Pose2d{1.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(first.x, replay.x);
  EXPECT_DOUBLE_EQ(first.y, replay.y);
  EXPECT_DOUBLE_EQ(first.yaw, replay.yaw);
}

}  // namespace embodied_slam
