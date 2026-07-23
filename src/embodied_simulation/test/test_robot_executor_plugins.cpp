#include <algorithm>
#include <string>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>

#include "embodied_simulation/robot_executor.hpp"

namespace embodied_simulation
{
namespace
{

TEST(RobotExecutorPluginsTest, GazeboAndMockAdaptersAreDiscoverable)
{
  pluginlib::ClassLoader<RobotExecutor> loader(
    "embodied_simulation", "embodied_simulation::RobotExecutor");
  const auto classes = loader.getDeclaredClasses();
  EXPECT_NE(
    std::find(
      classes.begin(), classes.end(),
      "embodied_simulation/GazeboRobotExecutor"),
    classes.end());
  EXPECT_NE(
    std::find(
      classes.begin(), classes.end(),
      "embodied_simulation/MockRobotExecutor"),
    classes.end());
  EXPECT_NE(
    std::find(
      classes.begin(), classes.end(),
      "embodied_simulation/Nav2RobotExecutor"),
    classes.end());
}

TEST(RobotExecutorPluginsTest, IdleManualExecutorRelinquishesCmdVelOwnership)
{
  EXPECT_FALSE(should_publish_executor_cmd_vel(true, false, false, "manual"));
  EXPECT_TRUE(should_publish_executor_cmd_vel(true, true, false, "manual"));
  EXPECT_TRUE(should_publish_executor_cmd_vel(true, false, true, "manual"));
  EXPECT_TRUE(
    should_publish_executor_cmd_vel(true, false, false, "wall_following"));
  EXPECT_FALSE(
    should_publish_executor_cmd_vel(false, true, true, "wall_following"));
}

TEST(RobotExecutorPluginsTest, SameCommandRunsThroughBothAdapters)
{
  pluginlib::ClassLoader<RobotExecutor> loader(
    "embodied_simulation", "embodied_simulation::RobotExecutor");
  auto gazebo = loader.createSharedInstance(
    "embodied_simulation/GazeboRobotExecutor");
  auto mock = loader.createSharedInstance(
    "embodied_simulation/MockRobotExecutor");
  gazebo->configure(ControllerConfig{});
  mock->configure(ControllerConfig{});

  std::vector<float> clear_scan(360, 2.0F);
  gazebo->update_scan(
    clear_scan, -3.14159265, 2.0 * 3.14159265 / 360.0,
    0.05, 10.0, 0.0);

  embodied_agent_interfaces::msg::RobotCommand command;
  command.action_type = command.MOVE;
  command.linear_x = 0.15;
  command.duration_s = 1.0;
  ASSERT_TRUE(gazebo->execute(command, 0.0));
  ASSERT_TRUE(mock->execute(command, 0.0));

  EXPECT_GT(gazebo->step(0.05).velocity.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(mock->step(0.05).velocity.linear_x, 0.15);
  EXPECT_EQ(gazebo->backend_name(), "simulation");
  EXPECT_EQ(mock->backend_name(), "mock");

  gazebo->stop();
  mock->stop();
  EXPECT_DOUBLE_EQ(gazebo->step(0.10).velocity.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(mock->step(0.10).velocity.linear_x, 0.0);
}

TEST(RobotExecutorPluginsTest, MoveCommandCanCarryArcVelocityForDemo)
{
  pluginlib::ClassLoader<RobotExecutor> loader(
    "embodied_simulation", "embodied_simulation::RobotExecutor");
  auto gazebo = loader.createSharedInstance(
    "embodied_simulation/GazeboRobotExecutor");
  auto mock = loader.createSharedInstance(
    "embodied_simulation/MockRobotExecutor");
  gazebo->configure(ControllerConfig{});
  mock->configure(ControllerConfig{});

  std::vector<float> clear_scan(360, 2.0F);
  gazebo->update_scan(
    clear_scan, -3.14159265, 2.0 * 3.14159265 / 360.0,
    0.05, 10.0, 0.0);

  embodied_agent_interfaces::msg::RobotCommand command;
  command.action_type = command.MOVE;
  command.linear_x = 0.12;
  command.angular_z = 0.45;
  command.duration_s = 6.0;

  ASSERT_TRUE(gazebo->execute(command, 0.0));
  ASSERT_TRUE(mock->execute(command, 0.0));
  EXPECT_GT(gazebo->step(0.05).velocity.linear_x, 0.0);
  EXPECT_GT(gazebo->step(0.05).velocity.angular_z, 0.0);
  EXPECT_DOUBLE_EQ(mock->step(0.05).velocity.linear_x, 0.12);
  EXPECT_DOUBLE_EQ(mock->step(0.05).velocity.angular_z, 0.45);
}

TEST(RobotExecutorPluginsTest, AccessoriesAreAcceptedAsSimulationAcks)
{
  pluginlib::ClassLoader<RobotExecutor> loader(
    "embodied_simulation", "embodied_simulation::RobotExecutor");
  auto mock = loader.createSharedInstance(
    "embodied_simulation/MockRobotExecutor");
  mock->configure(ControllerConfig{});

  embodied_agent_interfaces::msg::RobotCommand wave;
  wave.action_type = wave.WAVE;
  wave.count = 2;
  EXPECT_TRUE(mock->execute(wave, 0.0));

  embodied_agent_interfaces::msg::RobotCommand led;
  led.action_type = led.SET_LED;
  led.color = "blue";
  EXPECT_TRUE(mock->execute(led, 0.0));
  EXPECT_DOUBLE_EQ(mock->step(0.05).velocity.linear_x, 0.0);
}

TEST(RobotExecutorPluginsTest, NavigationCommandsProduceObservableMotion)
{
  pluginlib::ClassLoader<RobotExecutor> loader(
    "embodied_simulation", "embodied_simulation::RobotExecutor");
  auto gazebo = loader.createSharedInstance(
    "embodied_simulation/GazeboRobotExecutor");
  auto mock = loader.createSharedInstance(
    "embodied_simulation/MockRobotExecutor");
  gazebo->configure(ControllerConfig{});
  mock->configure(ControllerConfig{});

  std::vector<float> clear_scan(360, 2.0F);
  gazebo->update_scan(
    clear_scan, -3.14159265, 2.0 * 3.14159265 / 360.0,
    0.05, 10.0, 0.0);

  embodied_agent_interfaces::msg::RobotCommand nav;
  nav.action_type = nav.NAVIGATE_TO;
  nav.target = "door";
  nav.duration_s = 3.0;
  ASSERT_TRUE(gazebo->execute(nav, 0.0));
  ASSERT_TRUE(mock->execute(nav, 0.0));
  EXPECT_GT(gazebo->step(0.05).velocity.linear_x, 0.0);
  EXPECT_GT(mock->step(0.05).velocity.linear_x, 0.0);

  embodied_agent_interfaces::msg::RobotCommand patrol;
  patrol.action_type = patrol.FOLLOW_WAYPOINTS;
  patrol.waypoints = {"door", "desk", "home"};
  patrol.number_of_loops = 1;
  patrol.duration_s = 6.0;
  ASSERT_TRUE(gazebo->execute(patrol, 1.0));
  ASSERT_TRUE(mock->execute(patrol, 1.0));
  EXPECT_GT(gazebo->step(1.05).velocity.angular_z, 0.0);
  EXPECT_GT(mock->step(1.05).velocity.angular_z, 0.0);

  embodied_agent_interfaces::msg::RobotCommand cancel;
  cancel.action_type = cancel.CANCEL_NAVIGATION;
  ASSERT_TRUE(gazebo->execute(cancel, 2.0));
  ASSERT_TRUE(mock->execute(cancel, 2.0));
  EXPECT_DOUBLE_EQ(gazebo->step(2.05).velocity.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(mock->step(2.05).velocity.linear_x, 0.0);
}

}  // namespace
}  // namespace embodied_simulation
