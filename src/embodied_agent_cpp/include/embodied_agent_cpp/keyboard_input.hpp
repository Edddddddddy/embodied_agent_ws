#pragma once

#include <chrono>
#include <cstdint>

namespace embodied_agent_cpp
{

enum class KeyboardIntent : std::uint8_t
{
  kNone = 0,
  kMotion,
  kSoftStop,
  kEmergencyStop,
  kResetAndResume,
  kSafeExit,
  kDeadmanStop,
  kBlockedByEmergencyStop,
};

struct KeyboardInputConfig
{
  double linear_speed{0.2};
  double angular_speed{0.8};
  double max_linear_speed{0.26};
  double max_angular_speed{1.82};
  // 600 ms 能覆盖终端按键自动重复的常见间隔，同时仍可在失焦/断连后快速停车。
  std::chrono::milliseconds deadman_timeout{600};
};

struct KeyboardInputUpdate
{
  KeyboardIntent intent{KeyboardIntent::kNone};
  double linear_x{0.0};
  double angular_z{0.0};
  bool recognized{false};
  bool emergency_stop_latched{false};
};

/// 将原始按键转换为确定性的运动意图；不依赖 ROS，便于对安全状态机做单元测试。
class KeyboardInput
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  explicit KeyboardInput(const KeyboardInputConfig & config = KeyboardInputConfig{});

  KeyboardInputUpdate apply_key(char key, TimePoint now);
  KeyboardInputUpdate update_deadman(TimePoint now);
  KeyboardInputUpdate snapshot() const;

  bool emergency_stop_latched() const noexcept;
  bool motion_active() const noexcept;

private:
  KeyboardInputUpdate make_update(KeyboardIntent intent, bool recognized) const;
  void set_motion(double linear_x, double angular_z, TimePoint now);
  void stop_motion();

  KeyboardInputConfig config_;
  double linear_x_{0.0};
  double angular_z_{0.0};
  bool motion_active_{false};
  bool emergency_stop_latched_{false};
  TimePoint last_motion_input_{};
};

const char * to_string(KeyboardIntent intent) noexcept;

}  // namespace embodied_agent_cpp
