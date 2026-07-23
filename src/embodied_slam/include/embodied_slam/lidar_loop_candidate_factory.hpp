#pragma once

#include <memory>

#include <rclcpp/node_options.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace embodied_slam
{

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_lidar_loop_candidate_node(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace embodied_slam
