#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string_view>
#include <unordered_set>

#include "embodied_agent_cpp/control_authority.hpp"

namespace embodied_agent_cpp
{

enum class AuthorityUpdateDecision : std::uint8_t
{
  kAcceptedInitial = 0,
  kAcceptedHeartbeat,
  kAcceptedTransition,
  kAcceptedManagerRestart,
  kRejectedInvalidPayload,
  kRejectedRetiredEpoch,
  kRejectedUnsafeEpochBootstrap,
  kRejectedLeaseDiscontinuity,
  kRejectedOldSequence,
  kRejectedConflictingHeartbeat,
  kRejectedReorderedReceipt,
};

struct AuthorityUpdateResult
{
  bool accepted{false};
  bool generation_changed{false};
  bool lease_discontinuity{false};
  AuthorityUpdateDecision decision{AuthorityUpdateDecision::kRejectedInvalidPayload};
};

/// 对 typed authority 状态执行 epoch、序号和本地 steady-clock 租约校验。
///
/// generation 是本进程的安全代际：状态迁移、manager 重启，或心跳中断超过租约
/// 后都会递增。动作缓冲只要绑定该代际，就不会在重新授权后复用失联前的旧动作。
class ControlAuthorityTracker
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit ControlAuthorityTracker(
    std::chrono::milliseconds lease = std::chrono::milliseconds{1000});

  AuthorityUpdateResult update(
    const ControlAuthoritySnapshot & snapshot,
    TimePoint received_at);

  bool observed() const noexcept;
  bool fresh(TimePoint now) const noexcept;
  bool fresh_autonomy(TimePoint now) const noexcept;
  std::uint64_t generation() const noexcept;
  ControlAuthority authority() const noexcept;
  const std::optional<ControlAuthoritySnapshot> & snapshot() const noexcept;

private:
  bool payload_is_valid(const ControlAuthoritySnapshot & snapshot) const noexcept;
  bool payload_matches(const ControlAuthoritySnapshot & snapshot) const noexcept;
  bool safe_manager_bootstrap(const ControlAuthoritySnapshot & snapshot) const noexcept;
  AuthorityUpdateResult accept(
    const ControlAuthoritySnapshot & snapshot,
    TimePoint received_at,
    AuthorityUpdateDecision decision,
    bool generation_changed,
    bool lease_discontinuity);

  std::chrono::milliseconds lease_;
  std::optional<ControlAuthoritySnapshot> state_;
  std::optional<TimePoint> received_at_;
  std::uint64_t generation_{0};
  std::unordered_set<std::uint64_t> retired_epochs_;
};

std::string_view authority_update_reason(AuthorityUpdateDecision decision) noexcept;

}  // namespace embodied_agent_cpp
