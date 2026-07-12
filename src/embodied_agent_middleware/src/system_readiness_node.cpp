#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <embodied_agent_interfaces/msg/component_health.hpp>
#include <embodied_agent_interfaces/msg/system_readiness.hpp>
#include <rclcpp/rclcpp.hpp>

#include "embodied_agent_middleware/component_health_registry.hpp"
#include "embodied_agent_middleware/qos_profiles.hpp"

namespace embodied_agent_middleware
{

class SystemReadinessNode : public rclcpp::Node
{
public:
  SystemReadinessNode()
  : Node("system_readiness")
  {
    profile_ = declare_parameter<std::string>("profile", "execution");
    required_components_ = parse_component_csv(declare_parameter<std::string>(
        "required_components_csv", "simulation_control,typed_action_bridge"));
    stale_timeout_s_ = std::max(
      0.1, declare_parameter<double>("stale_timeout_s", 3.0));
    const auto publish_period_ms = std::max<std::int64_t>(
      50, declare_parameter<int>("publish_period_ms", 250));

    health_sub_ = create_subscription<embodied_agent_interfaces::msg::ComponentHealth>(
      "system/component_health", state_qos(50),
      [this](const embodied_agent_interfaces::msg::ComponentHealth::SharedPtr message) {
        registry_.update({
          message->component,
          static_cast<ComponentState>(message->state),
          message->detail,
          steady_now_seconds()});
      });
    readiness_pub_ = create_publisher<embodied_agent_interfaces::msg::SystemReadiness>(
      "system/readiness", state_qos());
    timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms), [this]() {publish_readiness();});
    publish_readiness();
  }

private:
  double steady_now_seconds() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void publish_readiness()
  {
    const auto snapshot = registry_.evaluate(
      required_components_, steady_now_seconds(), stale_timeout_s_);
    embodied_agent_interfaces::msg::SystemReadiness message;
    message.stamp = now();
    message.profile = profile_;
    message.ready = snapshot.ready;
    message.required_components = snapshot.required_components;
    message.ready_components = snapshot.ready_components;
    message.missing_components = snapshot.missing_components;
    message.degraded_components = snapshot.degraded_components;
    message.detail = snapshot.detail;
    readiness_pub_->publish(message);
  }

  std::string profile_;
  std::vector<std::string> required_components_;
  double stale_timeout_s_{3.0};
  ComponentHealthRegistry registry_;
  rclcpp::Subscription<embodied_agent_interfaces::msg::ComponentHealth>::SharedPtr health_sub_;
  rclcpp::Publisher<embodied_agent_interfaces::msg::SystemReadiness>::SharedPtr readiness_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace embodied_agent_middleware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embodied_agent_middleware::SystemReadinessNode>());
  rclcpp::shutdown();
  return 0;
}
