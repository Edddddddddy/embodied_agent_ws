#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include "embodied_agent_interfaces/msg/control_authority_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "embodied_agent_cpp/control_authority.hpp"
#include "embodied_agent_cpp/velocity_authority_gate.hpp"
#include "embodied_agent_middleware/qos_profiles.hpp"

namespace embodied_agent_cpp
{
namespace
{

using AuthorityState = embodied_agent_interfaces::msg::ControlAuthorityState;
using Twist = geometry_msgs::msg::Twist;

std::optional<ControlAuthority> decode_authority(const std::uint8_t authority)
{
  switch (authority) {
    case AuthorityState::HOLD:
      return ControlAuthority::kHold;
    case AuthorityState::AUTONOMY:
      return ControlAuthority::kAutonomy;
    case AuthorityState::KEYBOARD:
      return ControlAuthority::kKeyboard;
    case AuthorityState::ESTOP:
      return ControlAuthority::kEstop;
    default:
      return std::nullopt;
  }
}

VelocityCommand from_message(const Twist & message)
{
  return VelocityCommand{
    message.linear.x,
    message.linear.y,
    message.linear.z,
    message.angular.x,
    message.angular.y,
    message.angular.z,
  };
}

Twist to_message(const VelocityCommand & velocity)
{
  Twist message;
  message.linear.x = velocity.linear_x;
  message.linear.y = velocity.linear_y;
  message.linear.z = velocity.linear_z;
  message.angular.x = velocity.angular_x;
  message.angular.y = velocity.angular_y;
  message.angular.z = velocity.angular_z;
  return message;
}

VelocityAuthorityGateConfig declare_gate_config(rclcpp::Node & node)
{
  VelocityAuthorityGateConfig config;
  config.authority_lease = std::chrono::milliseconds(
    node.declare_parameter(
      "authority_lease_ms", static_cast<std::int64_t>(config.authority_lease.count())));
  config.autonomy_timeout = std::chrono::milliseconds(
    node.declare_parameter(
      "autonomy_timeout_ms", static_cast<std::int64_t>(config.autonomy_timeout.count())));
  config.keyboard_timeout = std::chrono::milliseconds(
    node.declare_parameter(
      "keyboard_timeout_ms", static_cast<std::int64_t>(config.keyboard_timeout.count())));
  config.max_abs_linear_x = node.declare_parameter(
    "max_abs_linear_x", config.max_abs_linear_x);
  config.max_abs_angular_z = node.declare_parameter(
    "max_abs_angular_z", config.max_abs_angular_z);
  config.planar_axis_epsilon = node.declare_parameter(
    "planar_axis_epsilon", config.planar_axis_epsilon);
  config.expected_keyboard_source = node.declare_parameter(
    "expected_keyboard_source", config.expected_keyboard_source);
  return config;
}

}  // namespace

class VelocityAuthorityGateNode final : public rclcpp::Node
{
public:
  VelocityAuthorityGateNode()
  : Node("velocity_authority_gate"),
    gate_(declare_gate_config(*this)),
    publish_rate_hz_(declare_parameter("publish_rate_hz", 20.0)),
    autonomy_topic_(declare_parameter(
        "autonomy_topic", std::string("/control/autonomy/cmd_vel"))),
    keyboard_topic_(declare_parameter(
        "keyboard_topic", std::string("/control/keyboard/cmd_vel"))),
    authority_state_topic_(declare_parameter(
        "authority_state_topic", std::string("/control/authority/state"))),
    output_topic_(declare_parameter(
        "output_topic", std::string("/control/selected/cmd_vel")))
  {
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ < 5.0 || publish_rate_hz_ > 100.0) {
      throw std::invalid_argument("publish_rate_hz must be finite and in [5, 100]");
    }

    output_publisher_ = create_publisher<Twist>(
      output_topic_, embodied_agent_middleware::command_qos(10));
    autonomy_subscription_ = create_subscription<Twist>(
      autonomy_topic_,
      // Twist 没有 epoch/header；只保留最新一帧，避免 DDS 在 RESUME 后补送
      // 撤权前积压的速度。核心 Gate 仍会执行零速/静默隔离，QoS 只是第一层减载。
      embodied_agent_middleware::command_qos(1),
      [this](const Twist::SharedPtr message) {
        const auto now = VelocityAuthorityGate::Clock::now();
        {
          std::lock_guard<std::mutex> lock(gate_mutex_);
          if (!gate_.update_autonomy(from_message(*message), now)) {
            // RESUME 隔离期内拒绝旧非零 Twist 是正常安全行为。上游若持续发送，
            // 不能逐帧刷 ERROR 淹没真正故障；保留节流告警供现场排障。
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "autonomy velocity rejected: invalid, outside safety envelope or resume quarantine");
          }
        }
        evaluate_and_publish(now);
      });
    keyboard_subscription_ = create_subscription<Twist>(
      keyboard_topic_,
      embodied_agent_middleware::command_qos(1),
      [this](const Twist::SharedPtr message) {
        const auto now = VelocityAuthorityGate::Clock::now();
        {
          std::lock_guard<std::mutex> lock(gate_mutex_);
          if (!gate_.update_keyboard(from_message(*message), now)) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "keyboard velocity rejected: invalid, outside safety envelope or takeover quarantine");
          }
        }
        evaluate_and_publish(now);
      });
    authority_subscription_ = create_subscription<AuthorityState>(
      authority_state_topic_,
      embodied_agent_middleware::state_qos(),
      [this](const AuthorityState::SharedPtr message) {
        handle_authority(*message);
      });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    publish_timer_ = create_wall_timer(
      period,
      [this]() {
        // 必须用 steady_clock：Gazebo 暂停或 /clock 回拨也不能延长控制权租约。
        evaluate_and_publish(VelocityAuthorityGate::Clock::now());
      });

    // manager 尚未出现时也从第一帧开始持续发布零速，而不是等待某个输入触发。
    evaluate_and_publish(VelocityAuthorityGate::Clock::now());
    RCLCPP_INFO(
      get_logger(),
      "velocity authority gate ready: autonomy=%s keyboard=%s state=%s output=%s rate=%.1fHz",
      autonomy_topic_.c_str(), keyboard_topic_.c_str(), authority_state_topic_.c_str(),
      output_topic_.c_str(), publish_rate_hz_);
  }

private:
  void handle_authority(const AuthorityState & message)
  {
    const auto authority = decode_authority(message.authority);
    if (!authority) {
      RCLCPP_ERROR(
        get_logger(), "unknown authority value rejected: %u",
        static_cast<unsigned int>(message.authority));
      return;
    }

    ControlAuthoritySnapshot snapshot;
    snapshot.authority = *authority;
    snapshot.estop_latched = message.estop_latched;
    snapshot.manager_epoch = message.manager_epoch;
    snapshot.transition_sequence = message.transition_sequence;
    snapshot.pending_autonomy_revocation_sequence =
      message.pending_autonomy_revocation_sequence;
    snapshot.autonomy_quiescence_acknowledged =
      message.autonomy_quiescence_acknowledged;
    snapshot.active_source = message.active_source;
    snapshot.reason = message.reason;

    const auto now = VelocityAuthorityGate::Clock::now();
    bool accepted = false;
    {
      std::lock_guard<std::mutex> lock(gate_mutex_);
      accepted = gate_.update_authority(snapshot, now);
    }
    if (!accepted) {
      RCLCPP_ERROR(
        get_logger(),
        "authority state rejected: epoch=%llu sequence=%llu authority=%u source=%s",
        static_cast<unsigned long long>(message.manager_epoch),
        static_cast<unsigned long long>(message.transition_sequence),
        static_cast<unsigned int>(message.authority),
        message.active_source.c_str());
    }
    // 切到 HOLD/ESTOP 后立即发零，不等待下一个 timer tick。
    evaluate_and_publish(now);
  }

  void evaluate_and_publish(const VelocityAuthorityGate::TimePoint now)
  {
    VelocityGateOutput output;
    bool decision_changed = false;
    {
      std::lock_guard<std::mutex> lock(gate_mutex_);
      output = gate_.evaluate(now);
      decision_changed = !last_decision_ || *last_decision_ != output.decision;
      last_decision_ = output.decision;
    }
    output_publisher_->publish(to_message(output.velocity));
    if (decision_changed) {
      RCLCPP_INFO(
        get_logger(), "velocity gate decision=%s authorized=%s",
        to_string(output.decision), output.authorized ? "true" : "false");
    }
  }

  VelocityAuthorityGate gate_;
  double publish_rate_hz_;
  std::string autonomy_topic_;
  std::string keyboard_topic_;
  std::string authority_state_topic_;
  std::string output_topic_;

  std::mutex gate_mutex_;
  std::optional<VelocityGateDecision> last_decision_;
  rclcpp::Publisher<Twist>::SharedPtr output_publisher_;
  rclcpp::Subscription<Twist>::SharedPtr autonomy_subscription_;
  rclcpp::Subscription<Twist>::SharedPtr keyboard_subscription_;
  rclcpp::Subscription<AuthorityState>::SharedPtr authority_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace embodied_agent_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<embodied_agent_cpp::VelocityAuthorityGateNode>());
  } catch (const std::exception & error) {
    std::cerr << "velocity_authority_gate failed: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
