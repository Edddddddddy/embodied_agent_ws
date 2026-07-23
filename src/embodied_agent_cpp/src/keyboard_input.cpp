#include "embodied_agent_cpp/keyboard_input.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>

namespace embodied_agent_cpp
{
namespace
{

double checked_positive(double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and greater than zero");
  }
  return value;
}

}  // namespace

KeyboardInput::KeyboardInput(const KeyboardInputConfig & config)
: config_(config)
{
  config_.max_linear_speed = checked_positive(config_.max_linear_speed, "max_linear_speed");
  config_.max_angular_speed = checked_positive(config_.max_angular_speed, "max_angular_speed");
  if (!std::isfinite(config_.linear_speed) || !std::isfinite(config_.angular_speed)) {
    throw std::invalid_argument("keyboard speeds must be finite");
  }
  if (config_.deadman_timeout <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("deadman_timeout must be greater than zero");
  }

  // 速度参数也可能来自 launch/命令行，核心层再次限幅，避免绕过节点侧校验。
  config_.linear_speed = std::clamp(
    std::abs(config_.linear_speed), 0.0, config_.max_linear_speed);
  config_.angular_speed = std::clamp(
    std::abs(config_.angular_speed), 0.0, config_.max_angular_speed);
}

KeyboardInputUpdate KeyboardInput::apply_key(char key, TimePoint now)
{
  const char normalized = static_cast<char>(
    std::tolower(static_cast<unsigned char>(key)));

  if (normalized == 'x') {
    emergency_stop_latched_ = true;
    stop_motion();
    return make_update(KeyboardIntent::kEmergencyStop, true);
  }
  if (normalized == 'r') {
    emergency_stop_latched_ = false;
    stop_motion();
    return make_update(KeyboardIntent::kResetAndResume, true);
  }
  if (normalized == 'q') {
    stop_motion();
    return make_update(KeyboardIntent::kSafeExit, true);
  }
  if (key == ' ') {
    stop_motion();
    return make_update(KeyboardIntent::kSoftStop, true);
  }

  const bool is_motion_key =
    normalized == 'w' || normalized == 's' || normalized == 'a' || normalized == 'd';
  if (!is_motion_key) {
    return make_update(KeyboardIntent::kNone, false);
  }
  if (emergency_stop_latched_) {
    // 急停必须显式按 R 解锁；忽略后续运动键可防止按键缓存导致意外复驶。
    stop_motion();
    return make_update(KeyboardIntent::kBlockedByEmergencyStop, true);
  }

  switch (normalized) {
    case 'w':
      set_motion(config_.linear_speed, 0.0, now);
      break;
    case 's':
      set_motion(-config_.linear_speed, 0.0, now);
      break;
    case 'a':
      set_motion(0.0, config_.angular_speed, now);
      break;
    case 'd':
      set_motion(0.0, -config_.angular_speed, now);
      break;
    default:
      break;
  }
  return make_update(KeyboardIntent::kMotion, true);
}

KeyboardInputUpdate KeyboardInput::update_deadman(TimePoint now)
{
  if (
    motion_active_ &&
    now - last_motion_input_ >= config_.deadman_timeout)
  {
    // 终端焦点丢失或 SSH 断开时不会再有 key-up 事件，超时主动归零是最后一道保护。
    stop_motion();
    return make_update(KeyboardIntent::kDeadmanStop, true);
  }
  return snapshot();
}

KeyboardInputUpdate KeyboardInput::snapshot() const
{
  return make_update(KeyboardIntent::kNone, false);
}

bool KeyboardInput::emergency_stop_latched() const noexcept
{
  return emergency_stop_latched_;
}

bool KeyboardInput::motion_active() const noexcept
{
  return motion_active_;
}

KeyboardInputUpdate KeyboardInput::make_update(
  KeyboardIntent intent, bool recognized) const
{
  return KeyboardInputUpdate{
    intent,
    linear_x_,
    angular_z_,
    recognized,
    emergency_stop_latched_,
  };
}

void KeyboardInput::set_motion(double linear_x, double angular_z, TimePoint now)
{
  linear_x_ = std::clamp(linear_x, -config_.max_linear_speed, config_.max_linear_speed);
  angular_z_ = std::clamp(angular_z, -config_.max_angular_speed, config_.max_angular_speed);
  last_motion_input_ = now;
  motion_active_ = true;
}

void KeyboardInput::stop_motion()
{
  linear_x_ = 0.0;
  angular_z_ = 0.0;
  motion_active_ = false;
}

const char * to_string(KeyboardIntent intent) noexcept
{
  switch (intent) {
    case KeyboardIntent::kNone:
      return "none";
    case KeyboardIntent::kMotion:
      return "motion";
    case KeyboardIntent::kSoftStop:
      return "soft_stop";
    case KeyboardIntent::kEmergencyStop:
      return "emergency_stop";
    case KeyboardIntent::kResetAndResume:
      return "reset_and_resume";
    case KeyboardIntent::kSafeExit:
      return "safe_exit";
    case KeyboardIntent::kDeadmanStop:
      return "deadman_stop";
    case KeyboardIntent::kBlockedByEmergencyStop:
      return "blocked_by_emergency_stop";
  }
  return "unknown";
}

}  // namespace embodied_agent_cpp
