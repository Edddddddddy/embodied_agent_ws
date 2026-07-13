#include <gtest/gtest.h>

#include <cmath>

#include "embodied_slam/closed_loop_controller.hpp"

namespace embodied_slam
{

TEST(ClosedLoopController, DrivesFourSidesAndStops)
{
  ClosedLoopConfig config;
  config.segment_length_m = 1.0;
  config.position_tolerance_m = 0.01;
  config.angle_tolerance_rad = 0.01;
  ClosedLoopController controller(config);

  EXPECT_GT(controller.update(Pose2d{0.0, 0.0, 0.0}).linear_x, 0.0);
  controller.update(Pose2d{1.0, 0.0, 0.0});
  controller.update(Pose2d{1.0, 0.0, 1.5708});
  controller.update(Pose2d{1.0, 1.0, 1.5708});
  controller.update(Pose2d{1.0, 1.0, 3.14159});
  controller.update(Pose2d{0.0, 1.0, 3.14159});
  controller.update(Pose2d{0.0, 1.0, -1.5708});
  controller.update(Pose2d{0.0, 0.0, -1.5708});
  const auto complete = controller.update(Pose2d{0.0, 0.0, 0.0});
  EXPECT_TRUE(complete.complete);
  EXPECT_DOUBLE_EQ(complete.linear_x, 0.0);
  EXPECT_EQ(controller.completed_sides(), 4U);
}

TEST(ClosedLoopController, HandlesYawWrapDuringTurn)
{
  ClosedLoopConfig config;
  config.segment_length_m = 1.0;
  config.position_tolerance_m = 0.01;
  config.angle_tolerance_rad = 0.01;
  ClosedLoopController controller(config);
  controller.update(Pose2d{0.0, 0.0, 3.0});
  controller.update(Pose2d{-1.0, 0.0, 3.0});
  const auto before_wrap = controller.update(Pose2d{-1.0, 0.0, -3.0});
  EXPECT_GT(before_wrap.angular_z, 0.0);
  const auto after_turn = controller.update(Pose2d{-1.0, 0.0, -1.7124});
  EXPECT_GT(after_turn.linear_x, 0.0);
  EXPECT_EQ(controller.completed_sides(), 1U);
}

}  // namespace embodied_slam
