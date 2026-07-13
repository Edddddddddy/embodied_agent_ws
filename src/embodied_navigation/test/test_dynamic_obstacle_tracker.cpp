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

}  // namespace embodied_navigation
