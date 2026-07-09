#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "embodied_agent_cpp/summer_tts_service_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embodied_agent_cpp::SummerTtsServiceNode>());
  rclcpp::shutdown();
  return 0;
}
