#include <memory>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "embodied_simulation/simulation_control_factory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = embodied_simulation::make_simulation_control_node();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
