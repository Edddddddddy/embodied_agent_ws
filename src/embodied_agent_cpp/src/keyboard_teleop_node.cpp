#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>

#include "embodied_agent_interfaces/msg/control_authority_state.hpp"
#include "embodied_agent_interfaces/srv/set_control_authority.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "embodied_agent_cpp/control_authority.hpp"
#include "embodied_agent_cpp/control_authority_tracker.hpp"
#include "embodied_agent_cpp/keyboard_input.hpp"
#include "embodied_agent_middleware/qos_profiles.hpp"

namespace embodied_agent_cpp
{
namespace
{

using namespace std::chrono_literals;
using AuthorityState = embodied_agent_interfaces::msg::ControlAuthorityState;
using SetAuthority = embodied_agent_interfaces::srv::SetControlAuthority;

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

std::uint8_t encode_authority(const ControlAuthority authority)
{
  switch (authority) {
    case ControlAuthority::kHold:
      return AuthorityState::HOLD;
    case ControlAuthority::kAutonomy:
      return AuthorityState::AUTONOMY;
    case ControlAuthority::kKeyboard:
      return AuthorityState::KEYBOARD;
    case ControlAuthority::kEstop:
      return AuthorityState::ESTOP;
  }
  return AuthorityState::HOLD;
}

class TerminalRawMode final
{
public:
  explicit TerminalRawMode(int file_descriptor)
  : file_descriptor_(file_descriptor)
  {
    if (::isatty(file_descriptor_) != 1) {
      throw std::runtime_error("stdin is not a TTY; run keyboard_teleop in an interactive terminal");
    }
    if (::tcgetattr(file_descriptor_, &original_termios_) != 0) {
      throw std::system_error(errno, std::generic_category(), "tcgetattr");
    }
    original_flags_ = ::fcntl(file_descriptor_, F_GETFL, 0);
    if (original_flags_ < 0) {
      throw std::system_error(errno, std::generic_category(), "fcntl(F_GETFL)");
    }

    termios raw = original_termios_;
    // 保留 ISIG，让 Ctrl-C 仍由 ROS 正常处理；只关闭行缓冲和本地回显。
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_iflag &= static_cast<tcflag_t>(~(IXON | ICRNL));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (::tcsetattr(file_descriptor_, TCSANOW, &raw) != 0) {
      throw std::system_error(errno, std::generic_category(), "tcsetattr");
    }
    termios_changed_ = true;

    if (::fcntl(file_descriptor_, F_SETFL, original_flags_ | O_NONBLOCK) != 0) {
      restore();
      throw std::system_error(errno, std::generic_category(), "fcntl(F_SETFL)");
    }
    flags_changed_ = true;
  }

  ~TerminalRawMode()
  {
    restore();
  }

  TerminalRawMode(const TerminalRawMode &) = delete;
  TerminalRawMode & operator=(const TerminalRawMode &) = delete;

  std::optional<char> read_key()
  {
    char key = '\0';
    const ssize_t count = ::read(file_descriptor_, &key, 1);
    if (count == 1) {
      return key;
    }
    if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return std::nullopt;
    }
    throw std::system_error(errno, std::generic_category(), "read(stdin)");
  }

private:
  void restore() noexcept
  {
    if (flags_changed_) {
      (void)::fcntl(file_descriptor_, F_SETFL, original_flags_);
      flags_changed_ = false;
    }
    if (termios_changed_) {
      (void)::tcsetattr(file_descriptor_, TCSANOW, &original_termios_);
      termios_changed_ = false;
    }
  }

  int file_descriptor_;
  int original_flags_{0};
  termios original_termios_{};
  bool termios_changed_{false};
  bool flags_changed_{false};
};

KeyboardInputConfig declare_keyboard_config(rclcpp::Node & node)
{
  KeyboardInputConfig config;
  config.linear_speed = node.declare_parameter("linear_speed", config.linear_speed);
  config.angular_speed = node.declare_parameter("angular_speed", config.angular_speed);
  config.max_linear_speed =
    node.declare_parameter("max_linear_speed", config.max_linear_speed);
  config.max_angular_speed =
    node.declare_parameter("max_angular_speed", config.max_angular_speed);
  const auto deadman_timeout_ms = node.declare_parameter(
    "deadman_timeout_ms", static_cast<int64_t>(config.deadman_timeout.count()));
  config.deadman_timeout = std::chrono::milliseconds(deadman_timeout_ms);
  return config;
}

std::chrono::milliseconds declare_authority_lease(rclcpp::Node & node)
{
  return std::chrono::milliseconds(
    node.declare_parameter<std::int64_t>("authority_lease_ms", 750));
}

}  // namespace

class KeyboardTeleopNode final : public rclcpp::Node
{
public:
  KeyboardTeleopNode()
  : Node("keyboard_teleop"),
    keyboard_input_(declare_keyboard_config(*this)),
    authority_tracker_(declare_authority_lease(*this)),
    publish_rate_hz_(declare_parameter("publish_rate_hz", 20.0)),
    cmd_vel_topic_(declare_parameter(
        "cmd_vel_topic", std::string("/control/keyboard/cmd_vel"))),
    authority_service_(declare_parameter(
        "authority_service", std::string("/control/set_authority"))),
    authority_state_topic_(declare_parameter(
        "authority_state_topic", std::string("/control/authority/state"))),
    requester_(declare_parameter("requester", std::string("keyboard_teleop")))
  {
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ < 1.0 || publish_rate_hz_ > 100.0) {
      throw std::invalid_argument("publish_rate_hz must be finite and in [1, 100]");
    }
    terminal_ = std::make_unique<TerminalRawMode>(STDIN_FILENO);

    velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, embodied_agent_middleware::command_qos(10));
    authority_client_ = create_client<SetAuthority>(authority_service_);
    authority_subscription_ = create_subscription<AuthorityState>(
      authority_state_topic_,
      embodied_agent_middleware::state_qos(),
      [this](const AuthorityState::SharedPtr message) {
        update_authority_cache(*message);
      });

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    input_timer_ = create_wall_timer(timer_period, [this]() {process_input();});

    RCLCPP_INFO(
      get_logger(),
      "keyboard ready: topic=%s service=%s rate=%.1fHz (W/S A/D Space X R Q)",
      cmd_vel_topic_.c_str(), authority_service_.c_str(), publish_rate_hz_);
  }

  ~KeyboardTeleopNode() override
  {
    publish_zero();
  }

private:
  void process_input()
  {
    const auto now = KeyboardInput::TimePoint::clock::now();
    try {
      while (const auto key = terminal_->read_key()) {
        const bool was_estop_latched = keyboard_input_.emergency_stop_latched();
        const auto update = keyboard_input_.apply_key(*key, now);
        if (update.recognized) {
          handle_intent(update.intent, was_estop_latched);
        }
      }
    } catch (const std::system_error & error) {
      RCLCPP_ERROR(get_logger(), "keyboard read failed: %s", error.what());
      request_authority(SetAuthority::Request::ENTER_HOLD, "keyboard_read_failed");
      publish_zero();
      schedule_shutdown();
      return;
    }

    const auto deadman = keyboard_input_.update_deadman(now);
    if (deadman.intent == KeyboardIntent::kDeadmanStop) {
      RCLCPP_WARN(get_logger(), "keyboard deadman timeout: velocity forced to zero");
      request_authority(SetAuthority::Request::RELEASE_KEYBOARD, "keyboard_deadman_timeout");
    }

    const auto current = keyboard_input_.snapshot();
    if (!shutdown_scheduled_.load()) {
      const auto authority = authority_snapshot();
      if (
        keyboard_input_.motion_active() &&
        (
          authority.authority != AuthorityState::KEYBOARD ||
          authority.active_source != requester_))
      {
        maybe_take_keyboard();
      }
      // 不能只检查 KEYBOARD 枚举：另一个键盘 requester 接管时，本进程的缓存按键
      // 也必须保持零速。transition_sequence 与 active_source 一起构成授权快照。
      const bool keyboard_authorized =
        authority.seen &&
        authority.authority == AuthorityState::KEYBOARD &&
        authority.active_source == requester_ &&
        !release_request_pending_.load();
      publish_velocity(
        keyboard_authorized ? current.linear_x : 0.0,
        keyboard_authorized ? current.angular_z : 0.0);
    }
  }

  void handle_intent(KeyboardIntent intent, bool was_estop_latched)
  {
    switch (intent) {
      case KeyboardIntent::kMotion:
        maybe_take_keyboard();
        break;
      case KeyboardIntent::kSoftStop:
        publish_zero();
        // Space 是无条件进入 HOLD 的安全操作；即使当前并非键盘所有者也能终止自治。
        request_authority(SetAuthority::Request::ENTER_HOLD, "keyboard_soft_stop");
        break;
      case KeyboardIntent::kEmergencyStop:
        publish_zero();
        request_authority(SetAuthority::Request::EMERGENCY_STOP, "keyboard_emergency_stop");
        break;
      case KeyboardIntent::kResetAndResume:
        publish_zero();
        if (
          was_estop_latched ||
          authority_snapshot().authority == AuthorityState::ESTOP)
        {
          // 第一次 R 仅解除锁存，仍保持 HOLD；第二次 R 才恢复自治，避免急停后突然复驶。
          request_authority(
            SetAuthority::Request::RESET_EMERGENCY_STOP, "keyboard_reset_emergency_stop");
        } else {
          request_authority(SetAuthority::Request::RESUME_AUTONOMY, "keyboard_resume_autonomy");
        }
        break;
      case KeyboardIntent::kSafeExit:
        publish_zero();
        request_authority(SetAuthority::Request::ENTER_HOLD, "keyboard_safe_exit");
        schedule_shutdown();
        break;
      case KeyboardIntent::kBlockedByEmergencyStop:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "motion key ignored: emergency stop is latched; press R to reset");
        break;
      case KeyboardIntent::kDeadmanStop:
      case KeyboardIntent::kNone:
        break;
    }
  }

  void maybe_take_keyboard()
  {
    const auto authority = authority_snapshot();
    if (
      (
        authority.authority == AuthorityState::KEYBOARD &&
        authority.active_source == requester_) ||
      take_request_pending_.load() ||
      release_request_pending_.load())
    {
      return;
    }
    request_authority(SetAuthority::Request::TAKE_KEYBOARD, "keyboard_motion_input", true);
  }

  void request_authority(
    std::uint8_t command, const std::string & reason, bool is_take_request = false)
  {
    if (!authority_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "authority service %s is not ready; non-zero keyboard velocity remains gated",
        authority_service_.c_str());
      return;
    }

    auto request = std::make_shared<SetAuthority::Request>();
    request->command = command;
    request->requester = requester_;
    request->reason = reason;
    const bool is_release_request = command == SetAuthority::Request::RELEASE_KEYBOARD;
    if (is_take_request) {
      take_request_pending_.store(true);
    }
    if (is_release_request) {
      // deadman 松权和下一次运动键可能紧邻发生；等待 RELEASE 完成后再 TAKE，避免请求乱序。
      release_request_pending_.store(true);
    }
    try {
      authority_client_->async_send_request(
        request,
        [this, command, is_take_request, is_release_request](
          rclcpp::Client<SetAuthority>::SharedFuture future)
        {
          if (is_take_request) {
            take_request_pending_.store(false);
          }
          if (is_release_request) {
            release_request_pending_.store(false);
          }
          try {
            const auto response = future.get();
            if (!response->accepted) {
              RCLCPP_WARN(
                get_logger(), "authority command %u rejected: %s",
                static_cast<unsigned int>(command), response->message.c_str());
            } else {
              update_authority_cache(response->state);
              RCLCPP_INFO(
                get_logger(), "authority command %u accepted: %s",
                static_cast<unsigned int>(command), response->message.c_str());
            }
          } catch (const std::exception & error) {
            RCLCPP_ERROR(get_logger(), "authority request failed: %s", error.what());
          }
          if (is_release_request && keyboard_input_.motion_active()) {
            maybe_take_keyboard();
          }
        });
    } catch (const std::exception & error) {
      take_request_pending_.store(false);
      release_request_pending_.store(false);
      RCLCPP_ERROR(get_logger(), "failed to send authority request: %s", error.what());
    }
  }

  void publish_velocity(double linear_x, double angular_z)
  {
    geometry_msgs::msg::Twist message;
    message.linear.x = linear_x;
    message.angular.z = angular_z;
    velocity_publisher_->publish(message);
  }

  void publish_zero()
  {
    if (velocity_publisher_) {
      publish_velocity(0.0, 0.0);
    }
  }

  void schedule_shutdown()
  {
    if (shutdown_scheduled_.exchange(true)) {
      return;
    }
    publish_zero();
    shutdown_timer_ = create_wall_timer(150ms, [this]() {
        shutdown_timer_->cancel();
        publish_zero();
        rclcpp::shutdown();
      });
  }

  struct CachedAuthority
  {
    std::uint8_t authority{AuthorityState::HOLD};
    std::uint64_t manager_epoch{0};
    std::uint64_t transition_sequence{0};
    std::string active_source;
    bool seen{false};
  };

  CachedAuthority authority_snapshot() const
  {
    std::lock_guard<std::mutex> lock(authority_mutex_);
    const auto now = ControlAuthorityTracker::Clock::now();
    if (!authority_tracker_.fresh(now) || !authority_tracker_.snapshot()) {
      return CachedAuthority{};
    }
    const auto & snapshot = *authority_tracker_.snapshot();
    return CachedAuthority{
      encode_authority(snapshot.authority),
      snapshot.manager_epoch,
      snapshot.transition_sequence,
      snapshot.active_source,
      true};
  }

  void update_authority_cache(const AuthorityState & message)
  {
    const auto authority = decode_authority(message.authority);
    if (!authority) {
      RCLCPP_ERROR(
        get_logger(), "unknown authority value ignored: %u",
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

    std::lock_guard<std::mutex> lock(authority_mutex_);
    // 与 ActionGuard/VelocityGate 共用 retired-epoch、payload 和 sticky lease
    // 规则；键盘缓存不能成为旧 manager 通过 ABA 序列回滚控制权的旁路。
    const auto update = authority_tracker_.update(
      snapshot, ControlAuthorityTracker::Clock::now());
    if (!update.accepted) {
      RCLCPP_ERROR(
        get_logger(),
        "authority state ignored: epoch=%llu sequence=%llu reason=%s",
        static_cast<unsigned long long>(message.manager_epoch),
        static_cast<unsigned long long>(message.transition_sequence),
        std::string(authority_update_reason(update.decision)).c_str());
    }
  }

  KeyboardInput keyboard_input_;
  mutable ControlAuthorityTracker authority_tracker_;
  double publish_rate_hz_;
  std::string cmd_vel_topic_;
  std::string authority_service_;
  std::string authority_state_topic_;
  std::string requester_;
  std::unique_ptr<TerminalRawMode> terminal_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Client<SetAuthority>::SharedPtr authority_client_;
  rclcpp::Subscription<AuthorityState>::SharedPtr authority_subscription_;
  rclcpp::TimerBase::SharedPtr input_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;

  mutable std::mutex authority_mutex_;
  std::atomic<bool> take_request_pending_{false};
  std::atomic<bool> release_request_pending_{false};
  std::atomic<bool> shutdown_scheduled_{false};
};

}  // namespace embodied_agent_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<embodied_agent_cpp::KeyboardTeleopNode>());
  } catch (const std::exception & error) {
    std::cerr << "keyboard_teleop failed: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
