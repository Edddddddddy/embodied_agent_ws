#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "embodied_slam/lidar_loop_constraint_gate_factory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = embodied_slam::makeLidarLoopConstraintGateNode();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
