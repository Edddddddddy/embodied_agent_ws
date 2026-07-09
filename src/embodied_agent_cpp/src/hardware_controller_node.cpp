#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "embodied_agent_interfaces/msg/robot_command.hpp"
#include "embodied_agent_cpp/hardware_transport.hpp"
#include "embodied_agent_cpp/hardware_protocol.hpp"

namespace embodied_agent_cpp
{

class HardwareControllerNode : public rclcpp::Node
{
public:
  HardwareControllerNode()
  : Node("hardware_controller")
  {
    declare_parameter("backend", "mock");
    declare_parameter("uart_device", "/dev/ttyUSB0");
    declare_parameter("uart_baud_rate", 115200);
    declare_parameter("spi_device", "/dev/spidev0.0");
    declare_parameter("spi_speed_hz", 1000000);
    const auto backend = get_parameter("backend").as_string();
    if (backend == "mock") {
      transport_ = make_mock_transport();
    } else if (backend == "uart") {
      transport_ = make_uart_transport(
        get_parameter("uart_device").as_string(),
        static_cast<int>(get_parameter("uart_baud_rate").as_int()));
    } else if (backend == "spi") {
      transport_ = make_spi_transport(
        get_parameter("spi_device").as_string(),
        static_cast<std::uint32_t>(get_parameter("spi_speed_hz").as_int()));
    } else {
      throw std::invalid_argument("backend must be mock, uart, or spi");
    }

    std::string error;
    if (!transport_->open(error)) {
      throw std::runtime_error("hardware transport initialization failed: " + error);
    }
    ack_publisher_ = create_publisher<std_msgs::msg::String>("/robot/action_ack", 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>("/robot/hardware_status", 10);
    command_subscription_ =
      create_subscription<embodied_agent_interfaces::msg::RobotCommand>(
      "/robot/action_command_typed", 10,
      [this](const embodied_agent_interfaces::msg::RobotCommand::SharedPtr message) {
        execute(*message);
      });
    emergency_subscription_ = create_subscription<std_msgs::msg::Empty>(
      "/robot/emergency_stop", 10,
      [this](const std_msgs::msg::Empty::SharedPtr) {send_stop("emergency_stop");});
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() {
        if (watchdog_.expired(MotionWatchdog::Clock::now())) {
          send_stop("duration_elapsed");
        }
      });
    publish_status("ready", "");
    RCLCPP_INFO(get_logger(), "hardware controller ready: %s", transport_->name().c_str());
  }

private:
  void execute(const embodied_agent_interfaces::msg::RobotCommand & command)
  {
    if (command.action_type == command.STOP) {
      send_stop(command.source.empty() ? "typed_command" : command.source);
      return;
    }
    const auto encoded = protocol_.encode(to_protocol_command(command).dump());
    if (!encoded.valid) {
      publish_status("rejected", encoded.error);
      return;
    }
    std::string error;
    if (!transport_->send(encoded.frame, error)) {
      publish_status("io_error", error);
      RCLCPP_ERROR(get_logger(), "hardware send failed: %s", error.c_str());
      return;
    }
    if (encoded.action_name == "move" || encoded.action_name == "turn") {
      watchdog_.arm(encoded.motion_duration, MotionWatchdog::Clock::now());
    }
    publish_ack(encoded, command.source.empty() ? "typed_command" : command.source);
  }

  nlohmann::json to_protocol_command(
    const embodied_agent_interfaces::msg::RobotCommand & command) const
  {
    switch (command.action_type) {
      case embodied_agent_interfaces::msg::RobotCommand::MOVE:
        return {
          {"name", "move"},
          {"arguments", {
            {"linear_x", command.linear_x},
            {"duration_s", command.duration_s},
          }},
        };
      case embodied_agent_interfaces::msg::RobotCommand::TURN:
        return {
          {"name", "turn"},
          {"arguments", {
            {"angular_z", command.angular_z},
            {"duration_s", command.duration_s},
          }},
        };
      case embodied_agent_interfaces::msg::RobotCommand::WAVE:
        return {{"name", "wave"}, {"arguments", {{"count", command.count}}}};
      case embodied_agent_interfaces::msg::RobotCommand::SET_LED:
        return {
          {"name", "set_led"},
          {"arguments", {{"color", command.color}}},
        };
      default:
        return {
          {"name", "unsupported"},
          {"arguments", nlohmann::json::object()},
        };
    }
  }

  void send_stop(const std::string & source)
  {
    watchdog_.disarm();
    const auto stop = protocol_.encode_stop();
    std::string error;
    if (!transport_->send(stop.frame, error)) {
      publish_status("io_error", error);
      watchdog_.arm(std::chrono::milliseconds(100), MotionWatchdog::Clock::now());
      return;
    }
    publish_ack(stop, source);
  }

  void publish_ack(const EncodedHardwareCommand & command, const std::string & source)
  {
    nlohmann::json payload{
      {"status", "sent"}, {"action", command.action_name},
      {"sequence", command.sequence}, {"source", source},
      {"transport", transport_->name()},
    };
    std_msgs::msg::String message;
    message.data = payload.dump();
    ack_publisher_->publish(message);
  }

  void publish_status(const std::string & state, const std::string & detail)
  {
    nlohmann::json payload{{"state", state}, {"transport", transport_->name()}};
    if (!detail.empty()) {payload["detail"] = detail;}
    std_msgs::msg::String message;
    message.data = payload.dump();
    status_publisher_->publish(message);
  }

  HardwareProtocol protocol_;
  MotionWatchdog watchdog_;
  std::unique_ptr<HardwareTransport> transport_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ack_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<embodied_agent_interfaces::msg::RobotCommand>::SharedPtr
    command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace embodied_agent_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<embodied_agent_cpp::HardwareControllerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("hardware_controller"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
