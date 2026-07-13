#pragma once

#include <memory>

#include <rclcpp/node_options.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace embodied_agent_cpp
{

/// 为独立进程和 component container 提供同一个 Lifecycle 节点实现。
std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_typed_action_bridge_node(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace embodied_agent_cpp
