#include <gtest/gtest.h>

#include "embodied_navigation/dynamic_obstacle_tracker.hpp"

namespace embodied_navigation
{

TEST(DynamicObstacleTracker, EstimatesSmoothedConstantVelocity)
{
  TrackerConfig config;
  config.velocity_smoothing = 1.0;
  DynamicObstacleTracker tracker(config);
  tracker.update({Point2d{0.0, 0.0}}, 1.0);
  const auto & tracks = tracker.update({Point2d{0.5, 0.0}}, 2.0);
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_NEAR(tracks[0].velocity.x, 0.5, 1e-9);
  EXPECT_NEAR(tracks[0].velocity.y, 0.0, 1e-9);
  EXPECT_EQ(tracks[0].id, "track_1");
}

TEST(DynamicObstacleTracker, AssociatesNearestObservationsWithoutSwappingIds)
{
  DynamicObstacleTracker tracker;
  tracker.update({Point2d{-1.0, 0.0}, Point2d{1.0, 0.0}}, 1.0);
  const auto & tracks = tracker.update({Point2d{0.8, 0.0}, Point2d{-0.8, 0.0}}, 2.0);
  ASSERT_EQ(tracks.size(), 2U);
  EXPECT_EQ(tracks[0].id, "track_1");
  EXPECT_NEAR(tracks[0].position.x, -0.8, 1e-9);
  EXPECT_EQ(tracks[1].id, "track_2");
  EXPECT_NEAR(tracks[1].position.x, 0.8, 1e-9);
}

TEST(DynamicObstacleTracker, ExpiresStaleTracksAndStartsANewIdentity)
{
  TrackerConfig config;
  config.track_timeout_s = 0.5;
  DynamicObstacleTracker tracker(config);
  tracker.update({Point2d{0.0, 0.0}}, 1.0);
  EXPECT_TRUE(tracker.update({}, 1.7).empty());
  const auto & tracks = tracker.update({Point2d{0.0, 0.0}}, 1.8);
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_EQ(tracks[0].id, "track_2");
}

TEST(DynamicObstacleTracker, ParsesAllSupportedMotionModels)
{
  EXPECT_EQ(motion_model_from_string("current_only"), MotionModel::CurrentOnly);
  EXPECT_EQ(motion_model_from_string("constant_velocity"), MotionModel::ConstantVelocity);
  EXPECT_EQ(motion_model_from_string("cv"), MotionModel::ConstantVelocity);
  EXPECT_EQ(motion_model_from_string("kalman"), MotionModel::Kalman);
  EXPECT_EQ(motion_model_from_string("imm"), MotionModel::Imm);
  EXPECT_THROW(motion_model_from_string("magic"), std::invalid_argument);
}

TEST(DynamicObstacleTracker, GlobalAssociationPreservesTwoExistingIdentities)
{
  TrackerConfig config;
  config.motion_model = MotionModel::CurrentOnly;
  config.association_distance_m = 0.5;
  config.association_strategy = AssociationStrategy::GlobalNearest;
  DynamicObstacleTracker tracker(config);
  tracker.update({Point2d{0.0, 0.0}, Point2d{0.3, 0.0}}, 0.0);

  const auto & tracks = tracker.update({Point2d{0.2, 0.0}, Point2d{-0.2, 0.0}}, 0.1);

  ASSERT_EQ(tracks.size(), 2U);
  EXPECT_EQ(tracks[0].id, "track_1");
  EXPECT_NEAR(tracks[0].position.x, -0.2, 1e-12);
  EXPECT_EQ(tracks[1].id, "track_2");
  EXPECT_NEAR(tracks[1].position.x, 0.2, 1e-12);
}

TEST(DynamicObstacleTracker, CurrentOnlyDoesNotInventVelocityDuringOcclusion)
{
  TrackerConfig config;
  config.motion_model = MotionModel::CurrentOnly;
  DynamicObstacleTracker tracker(config);
  tracker.update({Point2d{0.0, 0.0}}, 0.0);
  tracker.update({Point2d{1.0, 0.0}}, 1.0);
  const auto & tracks = tracker.update({}, 1.4);
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_DOUBLE_EQ(tracks[0].position.x, 1.0);
  EXPECT_DOUBLE_EQ(tracks[0].velocity.x, 0.0);
}

TEST(DynamicObstacleTracker, KalmanRejectsAlternatingMeasurementNoise)
{
  TrackerConfig config;
  config.motion_model = MotionModel::Kalman;
  config.measurement_noise_variance = 0.04;
  config.process_noise_variance = 0.05;
  DynamicObstacleTracker tracker(config);
  for (int index = 0; index <= 30; ++index) {
    const double time_s = 0.1 * static_cast<double>(index);
    const double noise = index % 2 == 0 ? 0.15 : -0.15;
    tracker.update({Point2d{time_s + noise, 0.0}}, time_s);
  }
  const auto & tracks = tracker.tracks();
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_NEAR(tracks[0].position.x, 3.0, 0.12);
  EXPECT_NEAR(tracks[0].velocity.x, 1.0, 0.15);
}

TEST(DynamicObstacleTracker, ImmRetainsTrackAcrossShortOcclusionAndAdaptsToStop)
{
  TrackerConfig config;
  config.motion_model = MotionModel::Imm;
  config.track_timeout_s = 1.0;
  config.measurement_noise_variance = 0.0025;
  DynamicObstacleTracker tracker(config);
  for (int index = 0; index <= 20; ++index) {
    const double time_s = 0.1 * static_cast<double>(index);
    tracker.update({Point2d{0.5 * time_s, 0.0}}, time_s);
  }
  const auto & occluded = tracker.update({}, 2.4);
  ASSERT_EQ(occluded.size(), 1U);
  EXPECT_EQ(occluded[0].id, "track_1");
  EXPECT_GT(occluded[0].position.x, 1.05);

  for (int index = 25; index <= 45; ++index) {
    const double time_s = 0.1 * static_cast<double>(index);
    tracker.update({Point2d{1.25, 0.0}}, time_s);
  }
  const auto & stopped = tracker.tracks();
  ASSERT_EQ(stopped.size(), 1U);
  EXPECT_NEAR(stopped[0].velocity.x, 0.0, 0.12);
}

}  // namespace embodied_navigation
