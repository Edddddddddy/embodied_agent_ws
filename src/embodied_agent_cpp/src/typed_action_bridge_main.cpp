#include <memory>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "embodied_agent_cpp/typed_action_bridge_factory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = embodied_agent_cpp::make_typed_action_bridge_node();
  // Action result、命令入口和 diagnostics 分属显式 callback group，需多线程执行器并发推进。
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 3);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
