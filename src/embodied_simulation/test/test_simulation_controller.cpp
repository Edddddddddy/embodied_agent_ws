#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "embodied_simulation/simulation_controller.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

std::vector<float> make_scan(float front, float left, float right)
{
  std::vector<float> ranges(360, 3.5F);
  ranges[180] = front;
  ranges[270] = left;
  ranges[90] = right;
  return ranges;
}

void update_scan(
  embodied_simulation::SimulationController & controller,
  float front, float left, float right, double now_s)
{
  controller.update_scan(
    make_scan(front, left, right), -kPi, kPi / 180.0, 0.12, 3.5, now_s);
}
}

TEST(SimulationControllerTest, ManualCommandExpiresAndIsSpeedLimited)
{
  embodied_simulation::SimulationController controller;
  update_scan(controller, 3.0F, 3.0F, 3.0F, 0.0);
  controller.set_manual_command(2.0, 0.0, 1.0, 0.0);
  auto output = controller.step(0.1);
  EXPECT_GT(output.velocity.linear_x, 0.0);
  EXPECT_LE(output.velocity.linear_x, 0.22);
  output = controller.step(1.1);
  EXPECT_LT(output.velocity.linear_x, 0.22);
}

TEST(SimulationControllerTest, EmergencyObstacleImmediatelyBlocksForwardMotion)
{
  embodied_simulation::SimulationController controller;
  controller.set_manual_command(0.2, 0.0, 2.0, 0.0);
  update_scan(controller, 0.15F, 1.0F, 1.0F, 0.1);
  const auto output = controller.step(0.1);
  EXPECT_DOUBLE_EQ(output.velocity.linear_x, 0.0);
  EXPECT_TRUE(output.safety_stopped);
  EXPECT_EQ(output.reason, "front_emergency");
}

TEST(SimulationControllerTest, ObstacleAvoidanceTurnsTowardClearerSide)
{
  embodied_simulation::SimulationController controller;
  ASSERT_TRUE(controller.set_mode("obstacle_avoidance"));
  update_scan(controller, 0.35F, 1.5F, 0.4F, 0.0);
  const auto output = controller.step(0.1);
  EXPECT_DOUBLE_EQ(output.velocity.linear_x, 0.0);
  EXPECT_GT(output.velocity.angular_z, 0.0);
  EXPECT_EQ(output.reason, "obstacle_turn");
}

TEST(SimulationControllerTest, AutonomousModesStopWhenScanIsStale)
{
  embodied_simulation::SimulationController controller;
  ASSERT_TRUE(controller.set_mode("obstacle_avoidance"));
  update_scan(controller, 2.0F, 2.0F, 2.0F, 0.0);
  EXPECT_GT(controller.step(0.1).velocity.linear_x, 0.0);
  const auto output = controller.step(1.0);
  EXPECT_DOUBLE_EQ(output.velocity.linear_x, 0.0);
  EXPECT_TRUE(output.sensor_stale);
  EXPECT_EQ(output.reason, "scan_timeout");
}

TEST(SimulationControllerTest, WallFollowerSteersAwayFromCloseRightWall)
{
  embodied_simulation::ControllerConfig config;
  config.wall_kd = 0.0;
  embodied_simulation::SimulationController controller(config);
  ASSERT_TRUE(controller.set_mode("wall_following"));
  update_scan(controller, 2.0F, 2.0F, 0.20F, 0.0);
  const auto output = controller.step(0.1);
  EXPECT_GT(output.velocity.linear_x, 0.0);
  EXPECT_GT(output.velocity.angular_z, 0.0);
  EXPECT_EQ(output.reason, "wall_tracking");
}

TEST(SimulationControllerTest, RejectsUnknownModeAndEmergencyStopLatches)
{
  embodied_simulation::SimulationController controller;
  EXPECT_FALSE(controller.set_mode("fly"));
  controller.set_manual_command(0.2, 0.0, 2.0, 0.0);
  controller.set_emergency_stop(true);
  EXPECT_DOUBLE_EQ(controller.step(0.1).velocity.linear_x, 0.0);
  controller.set_emergency_stop(false);
  EXPECT_DOUBLE_EQ(controller.step(0.2).velocity.linear_x, 0.0);
}

TEST(SimulationControllerTest, StopExitsAutonomousModeAndDoesNotResume)
{
  embodied_simulation::SimulationController controller;
  ASSERT_TRUE(controller.set_mode("obstacle_avoidance"));
  update_scan(controller, 2.0F, 2.0F, 2.0F, 0.0);
  EXPECT_GT(controller.step(0.1).velocity.linear_x, 0.0);
  controller.stop();
  EXPECT_EQ(controller.mode(), embodied_simulation::ControlMode::kManual);
  EXPECT_DOUBLE_EQ(controller.step(0.2).velocity.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(controller.step(0.3).velocity.linear_x, 0.0);
}

TEST(SimulationControllerTest, ManualForwardMotionStopsOnStaleScan)
{
  embodied_simulation::SimulationController controller;
  controller.set_manual_command(0.2, 0.0, 2.0, 0.0);
  const auto output = controller.step(1.0);
  EXPECT_DOUBLE_EQ(output.velocity.linear_x, 0.0);
  EXPECT_TRUE(output.safety_stopped);
  EXPECT_EQ(output.reason, "scan_timeout");
}
