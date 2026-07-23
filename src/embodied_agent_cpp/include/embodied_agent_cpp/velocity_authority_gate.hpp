#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "embodied_agent_cpp/control_authority.hpp"
#include "embodied_agent_cpp/control_authority_tracker.hpp"

namespace embodied_agent_cpp
{

/// 与 geometry_msgs/Twist 等价的 ROS 无关速度值，便于把安全门控完整放进单元测试。
struct VelocityCommand
{
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_x{0.0};
  double angular_y{0.0};
  double angular_z{0.0};
};

enum class VelocityGateDecision : std::uint8_t
{
  kAutonomyAuthorized = 0,
  kKeyboardAuthorized,
  kAuthorityUnseen,
  kAuthorityStale,
  kHold,
  kEmergencyStop,
  kAutonomyInputUnseen,
  kAutonomyInputStale,
  kAutonomyInputBeforeTransition,
  kAutonomyInputQuarantined,
  kKeyboardSourceMismatch,
  kKeyboardInputUnseen,
  kKeyboardInputStale,
  kKeyboardInputBeforeTransition,
  kKeyboardInputQuarantined,
};

struct VelocityAuthorityGateConfig
{
  std::chrono::milliseconds authority_lease{750};
  std::chrono::milliseconds autonomy_timeout{600};
  std::chrono::milliseconds keyboard_timeout{600};
  double max_abs_linear_x{0.26};
  double max_abs_angular_z{1.82};
  double planar_axis_epsilon{1e-9};
  std::string expected_keyboard_source{"keyboard_teleop"};
};

struct VelocityGateOutput
{
  VelocityCommand velocity;
  bool authorized{false};
  VelocityGateDecision decision{VelocityGateDecision::kAuthorityUnseen};
};

/// 最终速度权限门。
///
/// twist_mux 只负责 Nav2 与语音两个自动源的优先级；本类根据控制权状态，在自动源和
/// 键盘源之间做互斥授权。控制权与速度样本都使用 steady_clock 租约，仿真 /clock
/// 暂停或回拨时仍会可靠归零。
class VelocityAuthorityGate
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit VelocityAuthorityGate(
    const VelocityAuthorityGateConfig & config = VelocityAuthorityGateConfig{});

  /// 更新控制权心跳。旧序号或“相同序号但内容冲突”的状态不会刷新租约。
  bool update_authority(
    const ControlAuthoritySnapshot & snapshot,
    TimePoint received_at);
  bool update_autonomy(const VelocityCommand & velocity, TimePoint received_at);
  bool update_keyboard(const VelocityCommand & velocity, TimePoint received_at);

  VelocityGateOutput evaluate(TimePoint now) const;

private:
  struct TimedVelocity
  {
    VelocityCommand velocity;
    TimePoint received_at;
  };

  bool update_source(
    std::optional<TimedVelocity> & source,
    bool & quarantine_active,
    std::optional<TimePoint> & quarantine_quiet_since,
    const VelocityCommand & velocity,
    std::chrono::milliseconds timeout,
    TimePoint received_at);
  void start_source_quarantine(
    std::optional<TimedVelocity> & source,
    bool & quarantine_active,
    std::optional<TimePoint> & quarantine_quiet_since,
    TimePoint transition_at);
  VelocityGateOutput evaluate_source(
    const std::optional<TimedVelocity> & source,
    bool quarantine_active,
    std::chrono::milliseconds timeout,
    TimePoint now,
    VelocityGateDecision authorized,
    VelocityGateDecision unseen,
    VelocityGateDecision stale,
    VelocityGateDecision before_transition,
    VelocityGateDecision quarantined) const;

  VelocityAuthorityGateConfig config_;
  ControlAuthorityTracker authority_tracker_;
  std::optional<TimePoint> authority_transition_at_;
  std::optional<TimedVelocity> autonomy_;
  std::optional<TimedVelocity> keyboard_;
  bool autonomy_quarantine_active_{false};
  bool keyboard_quarantine_active_{false};
  std::optional<TimePoint> autonomy_quarantine_quiet_since_;
  std::optional<TimePoint> keyboard_quarantine_quiet_since_;
};

const char * to_string(VelocityGateDecision decision) noexcept;
bool is_finite(const VelocityCommand & velocity) noexcept;
bool is_zero(const VelocityCommand & velocity) noexcept;
bool is_planar(
  const VelocityCommand & velocity,
  double axis_epsilon) noexcept;
bool is_within_safety_envelope(
  const VelocityCommand & velocity,
  double max_abs_linear_x,
  double max_abs_angular_z) noexcept;

}  // namespace embodied_agent_cpp
