#include <gtest/gtest.h>

#include "embodied_simulation/node_configuration.hpp"

namespace embodied_simulation
{

TEST(NodeConfigurationTest, AcceptsSafeDefaults)
{
  EXPECT_TRUE(validate_node_configuration(
    ControllerConfig{}, 20.0, 12.0,
    "embodied_simulation/GazeboRobotExecutor").valid);
}

TEST(NodeConfigurationTest, RejectsInvalidTimingPluginAndSafetyThresholds)
{
  EXPECT_FALSE(validate_node_configuration(
    ControllerConfig{}, 0.0, 12.0, "plugin").valid);
  EXPECT_FALSE(validate_node_configuration(
    ControllerConfig{}, 20.0, 0.0, "plugin").valid);
  EXPECT_FALSE(validate_node_configuration(
    ControllerConfig{}, 20.0, 12.0, "").valid);

  ControllerConfig unsafe;
  unsafe.emergency_distance = 0.8;
  unsafe.obstacle_distance = 0.5;
  EXPECT_FALSE(validate_node_configuration(
    unsafe, 20.0, 12.0, "plugin").valid);

  ControllerConfig invalid_stop_timeout;
  invalid_stop_timeout.stop_timeout_s = 0.0;
  EXPECT_FALSE(validate_node_configuration(
    invalid_stop_timeout, 20.0, 12.0, "plugin").valid);
}

}  // namespace embodied_simulation
