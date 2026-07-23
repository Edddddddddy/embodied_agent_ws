#pragma once

#include <cstdint>
#include <string>

namespace embodied_agent_cpp
{

enum class ControlAuthority : std::uint8_t
{
  kHold = 0,
  kAutonomy = 1,
  kKeyboard = 2,
  kEstop = 3,
};

enum class ControlAuthorityCommand : std::uint8_t
{
  kTakeKeyboard = 1,
  kReleaseKeyboard = 2,
  kEnterHold = 3,
  kResumeAutonomy = 4,
  kEmergencyStop = 5,
  kResetEmergencyStop = 6,
};

struct ControlAuthoritySnapshot
{
  ControlAuthority authority{ControlAuthority::kHold};
  bool estop_latched{false};
  std::uint64_t manager_epoch{1};
  std::uint64_t transition_sequence{0};
  std::uint64_t pending_autonomy_revocation_sequence{0};
  bool autonomy_quiescence_acknowledged{true};
  std::string active_source;
  std::string reason{"initialized"};
};

struct ControlAuthorityRequest
{
  ControlAuthorityCommand command{ControlAuthorityCommand::kEnterHold};
  std::string requester;
  std::string reason;
};

struct AutonomyQuiescenceAcknowledgement
{
  std::uint64_t manager_epoch{0};
  std::uint64_t autonomy_revocation_sequence{0};
  std::string requester;
  std::string detail;
};

struct ControlAuthorityTransition
{
  bool accepted{false};
  bool changed{false};
  bool stop_requested{false};
  std::string message;
  ControlAuthoritySnapshot state;
};

/// 多控制源之间的纯 C++ 控制权状态机。
///
/// 该类型不依赖 rclcpp、时间或执行器。ROS 节点只负责把 typed service 转换为
/// request() 调用，并消费一次性的 stop_requested 边沿，因此安全语义可在单元测试
/// 中完整固定，不会被 DDS 重投递或 executor 时序改变。
class ControlAuthorityMachine
{
public:
  explicit ControlAuthorityMachine(
    std::uint64_t manager_epoch = 1U,
    bool bootstrap_quiescence_acknowledged = false);

  ControlAuthorityTransition request(const ControlAuthorityRequest & request);
  ControlAuthorityTransition acknowledge_autonomy_quiescence(
    const AutonomyQuiescenceAcknowledgement & acknowledgement);
  const ControlAuthoritySnapshot & snapshot() const noexcept;

private:
  ControlAuthorityTransition transition_to(
    ControlAuthority authority,
    const std::string & active_source,
    const std::string & reason,
    const std::string & message,
    bool stop_requested);
  ControlAuthorityTransition idempotent(const std::string & message) const;
  ControlAuthorityTransition reject(const std::string & message) const;

  ControlAuthoritySnapshot state_;
};

}  // namespace embodied_agent_cpp
