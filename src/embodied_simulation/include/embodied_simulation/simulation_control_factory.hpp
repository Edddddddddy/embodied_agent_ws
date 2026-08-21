#pragma once

#include <memory>

#include <rclcpp/node_options.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace embodied_simulation
{

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_simulation_control_node(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace embodied_simulation
