#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>

#include "embodied_agent_cpp/control_authority.hpp"
#include "embodied_agent_cpp/velocity_authority_gate.hpp"

namespace
{

using namespace std::chrono_literals;
using embodied_agent_cpp::ControlAuthority;
using embodied_agent_cpp::ControlAuthoritySnapshot;
using embodied_agent_cpp::VelocityAuthorityGate;
using embodied_agent_cpp::VelocityAuthorityGateConfig;
using embodied_agent_cpp::VelocityCommand;
using embodied_agent_cpp::VelocityGateDecision;
using embodied_agent_cpp::is_zero;

ControlAuthoritySnapshot state(
  const ControlAuthority authority,
  const std::uint64_t sequence,
  const std::string & active_source = "",
  const std::uint64_t manager_epoch = 1)
{
  ControlAuthoritySnapshot output;
  output.authority = authority;
  output.estop_latched = authority == ControlAuthority::kEstop;
  output.manager_epoch = manager_epoch;
  output.transition_sequence = sequence;
  output.pending_autonomy_revocation_sequence =
    authority == ControlAuthority::kAutonomy || sequence == 0U ?
    0U : sequence;
  output.autonomy_quiescence_acknowledged =
    authority != ControlAuthority::kAutonomy;
  output.active_source = active_source;
  output.reason =
    authority == ControlAuthority::kHold && sequence == 0U ?
    "initialized" : "test";
  return output;
}

VelocityCommand velocity(const double linear_x, const double angular_z = 0.0)
{
  VelocityCommand output;
  output.linear_x = linear_x;
  output.angular_z = angular_z;
  return output;
}

void acknowledge_zero(
  VelocityAuthorityGate & gate,
  const VelocityAuthorityGate::TimePoint at)
{
  ASSERT_TRUE(gate.update_autonomy(velocity(0.0), at));
}

TEST(VelocityAuthorityGateTest, UnseenAndStaleManagerContinuouslyFailClosed)
{
  VelocityAuthorityGate gate;
  const auto start = VelocityAuthorityGate::TimePoint{};

  auto output = gate.evaluate(start);
  EXPECT_FALSE(output.authorized);
  EXPECT_TRUE(is_zero(output.velocity));
  EXPECT_EQ(output.decision, VelocityGateDecision::kAuthorityUnseen);

  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kAutonomy, 1, "autonomy"), start));
  acknowledge_zero(gate, start + 1ms);
  ASSERT_TRUE(gate.update_autonomy(velocity(0.2), start + 2ms));
  output = gate.evaluate(start + 751ms);
  EXPECT_FALSE(output.authorized);
  EXPECT_TRUE(is_zero(output.velocity));
  EXPECT_EQ(output.decision, VelocityGateDecision::kAuthorityStale);

  // 同 epoch 的迟到心跳不能解除 stale；只能由新 manager 的 HOLD epoch 恢复。
  EXPECT_FALSE(gate.update_authority(
      state(ControlAuthority::kAutonomy, 1, "autonomy"), start + 800ms));
  EXPECT_EQ(
    gate.evaluate(start + 810ms).decision,
    VelocityGateDecision::kAuthorityStale);
}

TEST(VelocityAuthorityGateTest, AutonomyRequiresPostTransitionFreshSample)
{
  VelocityAuthorityGate gate;
  const auto start = VelocityAuthorityGate::TimePoint{};

  ASSERT_TRUE(gate.update_autonomy(velocity(0.2), start));
  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kAutonomy, 1, "autonomy"), start + 10ms));

  auto output = gate.evaluate(start + 20ms);
  EXPECT_FALSE(output.authorized);
  EXPECT_EQ(
    output.decision, VelocityGateDecision::kAutonomyInputQuarantined);

  // 模拟撤权前发布、恢复后才由 DDS 送达的旧非零速度：必须丢弃且重置静默窗。
  EXPECT_FALSE(gate.update_autonomy(velocity(0.2), start + 30ms));
  EXPECT_EQ(
    gate.evaluate(start + 31ms).decision,
    VelocityGateDecision::kAutonomyInputQuarantined);

  // 安全零速只解除隔离，不作为可复用缓存；其后的新样本才可执行。
  acknowledge_zero(gate, start + 40ms);
  EXPECT_EQ(
    gate.evaluate(start + 41ms).decision,
    VelocityGateDecision::kAutonomyInputUnseen);
  ASSERT_TRUE(gate.update_autonomy(velocity(0.25, 0.1), start + 50ms));
  output = gate.evaluate(start + 60ms);
  ASSERT_TRUE(output.authorized);
  EXPECT_EQ(output.decision, VelocityGateDecision::kAutonomyAuthorized);
  EXPECT_DOUBLE_EQ(output.velocity.linear_x, 0.25);
  EXPECT_DOUBLE_EQ(output.velocity.angular_z, 0.1);

  output = gate.evaluate(start + 651ms);
  EXPECT_FALSE(output.authorized);
  EXPECT_TRUE(is_zero(output.velocity));
  EXPECT_EQ(output.decision, VelocityGateDecision::kAutonomyInputStale);
}

TEST(VelocityAuthorityGateTest, KeyboardRequiresMatchingOwnerAndFreshSample)
{
  VelocityAuthorityGate gate;
  const auto start = VelocityAuthorityGate::TimePoint{};

  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kKeyboard, 1, "other_terminal"), start));
  ASSERT_TRUE(gate.update_keyboard(velocity(0.1), start + 10ms));
  auto output = gate.evaluate(start + 20ms);
  EXPECT_FALSE(output.authorized);
  EXPECT_EQ(output.decision, VelocityGateDecision::kKeyboardSourceMismatch);

  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kKeyboard, 2, "keyboard_teleop"), start + 30ms));
  output = gate.evaluate(start + 40ms);
  EXPECT_FALSE(output.authorized);
  EXPECT_EQ(
    output.decision, VelocityGateDecision::kKeyboardInputUnseen);

  ASSERT_TRUE(gate.update_keyboard(velocity(-0.2), start + 60ms));
  output = gate.evaluate(start + 70ms);
  ASSERT_TRUE(output.authorized);
  EXPECT_EQ(output.decision, VelocityGateDecision::kKeyboardAuthorized);
  EXPECT_DOUBLE_EQ(output.velocity.linear_x, -0.2);
}

TEST(VelocityAuthorityGateTest, HoldAndEmergencyStopNeverPassCachedVelocity)
{
  VelocityAuthorityGate gate;
  const auto start = VelocityAuthorityGate::TimePoint{};
  ASSERT_TRUE(gate.update_autonomy(velocity(0.2), start));
  ASSERT_TRUE(gate.update_keyboard(velocity(0.1), start));

  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kHold, 1), start));
  auto output = gate.evaluate(start + 10ms);
  EXPECT_EQ(output.decision, VelocityGateDecision::kHold);
  EXPECT_TRUE(is_zero(output.velocity));

  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kEstop, 2, "safety_panel"), start + 20ms));
  output = gate.evaluate(start + 30ms);
  EXPECT_EQ(output.decision, VelocityGateDecision::kEmergencyStop);
  EXPECT_TRUE(is_zero(output.velocity));
}

TEST(VelocityAuthorityGateTest, HeartbeatRefreshesLeaseWithoutRearmingOldVelocity)
{
  VelocityAuthorityGate gate;
  const auto start = VelocityAuthorityGate::TimePoint{};
  const auto autonomy = state(ControlAuthority::kAutonomy, 1, "autonomy");

  ASSERT_TRUE(gate.update_authority(autonomy, start));
  acknowledge_zero(gate, start + 50ms);
  ASSERT_TRUE(gate.update_autonomy(velocity(0.2), start + 100ms));
  ASSERT_TRUE(gate.update_authority(autonomy, start + 500ms));
  ASSERT_TRUE(gate.update_autonomy(velocity(0.24), start + 900ms));

  const auto output = gate.evaluate(start + 1000ms);
  ASSERT_TRUE(output.authorized);
  EXPECT_DOUBLE_EQ(output.velocity.linear_x, 0.24);
}

TEST(VelocityAuthorityGateTest, ReorderedOrConflictingAuthorityDoesNotRefreshLease)
{
  VelocityAuthorityGate gate;
  const auto start = VelocityAuthorityGate::TimePoint{};
  const auto autonomy = state(ControlAuthority::kAutonomy, 2, "autonomy");

  ASSERT_TRUE(gate.update_authority(autonomy, start));
  acknowledge_zero(gate, start + 50ms);
  ASSERT_TRUE(gate.update_autonomy(velocity(0.2), start + 100ms));
  EXPECT_FALSE(gate.update_authority(
      state(ControlAuthority::kKeyboard, 2, "keyboard_teleop"), start + 500ms));
  EXPECT_FALSE(gate.update_authority(
      state(ControlAuthority::kHold, 1), start + 600ms));

  const auto output = gate.evaluate(start + 751ms);
  EXPECT_FALSE(output.authorized);
  EXPECT_EQ(output.decision, VelocityGateDecision::kAuthorityStale);
}

TEST(VelocityAuthorityGateTest, NonFiniteInputInvalidatesTheSource)
{
  VelocityAuthorityGate gate;
  const auto start = VelocityAuthorityGate::TimePoint{};
  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kAutonomy, 1, "autonomy"), start));
  acknowledge_zero(gate, start + 5ms);
  ASSERT_TRUE(gate.update_autonomy(velocity(0.2), start + 10ms));

  auto invalid = velocity(0.2);
  invalid.angular_z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(gate.update_autonomy(invalid, start + 20ms));

  const auto output = gate.evaluate(start + 30ms);
  EXPECT_FALSE(output.authorized);
  EXPECT_EQ(output.decision, VelocityGateDecision::kAutonomyInputUnseen);
  EXPECT_TRUE(is_zero(output.velocity));
}

TEST(VelocityAuthorityGateTest, RejectsInconsistentEmergencyStopPayload)
{
  VelocityAuthorityGate gate;
  const auto start = VelocityAuthorityGate::TimePoint{};
  auto inconsistent = state(ControlAuthority::kEstop, 1, "safety_panel");
  inconsistent.estop_latched = false;

  EXPECT_FALSE(gate.update_authority(inconsistent, start));
  EXPECT_EQ(
    gate.evaluate(start).decision, VelocityGateDecision::kAuthorityUnseen);
}

TEST(VelocityAuthorityGateTest, ManagerRestartCreatesSafeEpochAndRejectsOldManager)
{
  VelocityAuthorityGate gate;
  const auto start = VelocityAuthorityGate::TimePoint{};
  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kAutonomy, 4, "autonomy", 10), start));
  acknowledge_zero(gate, start + 5ms);
  ASSERT_TRUE(gate.update_autonomy(velocity(0.2), start + 10ms));
  ASSERT_TRUE(gate.evaluate(start + 20ms).authorized);

  // 新进程必须先以 seq=0/HOLD/initialized 建立 epoch，重启瞬间不会复用旧速度。
  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kHold, 0, "", 20), start + 100ms));
  EXPECT_EQ(gate.evaluate(start + 110ms).decision, VelocityGateDecision::kHold);

  // 旧 epoch 不能先伪装成安全 bootstrap，再用较小的新序号完成 ABA 回滚。
  EXPECT_FALSE(gate.update_authority(
      state(ControlAuthority::kHold, 0, "", 10), start + 115ms));
  EXPECT_FALSE(gate.update_authority(
      state(ControlAuthority::kAutonomy, 5, "autonomy", 10), start + 120ms));
  EXPECT_EQ(gate.evaluate(start + 130ms).decision, VelocityGateDecision::kHold);

  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kAutonomy, 1, "autonomy", 20), start + 140ms));
  EXPECT_EQ(
    gate.evaluate(start + 150ms).decision,
    VelocityGateDecision::kAutonomyInputQuarantined);
  acknowledge_zero(gate, start + 155ms);
  ASSERT_TRUE(gate.update_autonomy(velocity(0.24), start + 160ms));
  EXPECT_TRUE(gate.evaluate(start + 170ms).authorized);
}

TEST(VelocityAuthorityGateTest, ResumeQuarantineRequiresZeroOrFullQuietBoundary)
{
  VelocityAuthorityGate gate;
  const auto start = VelocityAuthorityGate::TimePoint{};
  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kAutonomy, 1, "autonomy"), start));

  // 隔离期中的非零样本永不输出，并从该样本重新计算完整 source timeout。
  EXPECT_FALSE(gate.update_autonomy(velocity(0.2), start + 100ms));
  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kAutonomy, 1, "autonomy"), start + 500ms));
  EXPECT_FALSE(gate.update_autonomy(velocity(0.2), start + 699ms));
  EXPECT_EQ(
    gate.evaluate(start + 700ms).decision,
    VelocityGateDecision::kAutonomyInputQuarantined);

  // 静默满 600 ms 后的边界样本也必须丢弃；只有下一条确定更晚的样本可运动。
  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kAutonomy, 1, "autonomy"), start + 1000ms));
  EXPECT_FALSE(gate.update_autonomy(velocity(0.2), start + 1299ms));
  EXPECT_EQ(
    gate.evaluate(start + 1300ms).decision,
    VelocityGateDecision::kAutonomyInputUnseen);
  EXPECT_TRUE(gate.update_autonomy(velocity(0.2), start + 1310ms));
  const auto output = gate.evaluate(start + 1320ms);
  EXPECT_TRUE(output.authorized);
  EXPECT_DOUBLE_EQ(output.velocity.linear_x, 0.2);
}

TEST(VelocityAuthorityGateTest, InitialPayloadUsesUnifiedTrackerValidation)
{
  VelocityAuthorityGate gate;
  const auto start = VelocityAuthorityGate::TimePoint{};

  auto invalid_epoch = state(ControlAuthority::kHold, 0);
  invalid_epoch.manager_epoch = 0;
  EXPECT_FALSE(gate.update_authority(invalid_epoch, start));

  auto invalid_source = state(ControlAuthority::kHold, 0);
  invalid_source.active_source = "unexpected_owner";
  EXPECT_FALSE(gate.update_authority(invalid_source, start + 1ms));
  EXPECT_EQ(
    gate.evaluate(start + 2ms).decision,
    VelocityGateDecision::kAuthorityUnseen);
}

TEST(VelocityAuthorityGateTest, RejectsNonPlanarAxisAndClearsCachedMotion)
{
  VelocityAuthorityGate gate;
  const auto start = VelocityAuthorityGate::TimePoint{};
  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kAutonomy, 1, "autonomy"), start));
  acknowledge_zero(gate, start + 5ms);
  ASSERT_TRUE(gate.update_autonomy(velocity(0.2), start + 10ms));
  ASSERT_TRUE(gate.evaluate(start + 20ms).authorized);

  auto non_planar = velocity(0.2);
  non_planar.linear_y = 0.01;
  EXPECT_FALSE(gate.update_autonomy(non_planar, start + 30ms));
  const auto output = gate.evaluate(start + 40ms);
  EXPECT_FALSE(output.authorized);
  EXPECT_TRUE(is_zero(output.velocity));
  EXPECT_EQ(output.decision, VelocityGateDecision::kAutonomyInputUnseen);
}

TEST(VelocityAuthorityGateTest, EnforcesConfigurablePlanarSafetyEnvelope)
{
  VelocityAuthorityGateConfig config;
  config.max_abs_linear_x = 0.20;
  config.max_abs_angular_z = 1.00;
  VelocityAuthorityGate gate(config);
  const auto start = VelocityAuthorityGate::TimePoint{};
  ASSERT_TRUE(gate.update_authority(
      state(ControlAuthority::kAutonomy, 1, "autonomy"), start));
  acknowledge_zero(gate, start + 5ms);

  EXPECT_TRUE(gate.update_autonomy(velocity(0.20, -1.00), start + 10ms));
  auto output = gate.evaluate(start + 20ms);
  ASSERT_TRUE(output.authorized);
  EXPECT_DOUBLE_EQ(output.velocity.linear_x, 0.20);
  EXPECT_DOUBLE_EQ(output.velocity.angular_z, -1.00);

  EXPECT_FALSE(gate.update_autonomy(velocity(0.201), start + 30ms));
  EXPECT_TRUE(is_zero(gate.evaluate(start + 40ms).velocity));
  EXPECT_FALSE(gate.update_autonomy(velocity(0.1, 1.001), start + 50ms));
  EXPECT_TRUE(is_zero(gate.evaluate(start + 60ms).velocity));
}

TEST(VelocityAuthorityGateTest, RejectsInvalidSafetyEnvelopeConfiguration)
{
  VelocityAuthorityGateConfig config;
  config.max_abs_linear_x = 0.0;
  EXPECT_THROW((void)VelocityAuthorityGate{config}, std::invalid_argument);

  config.max_abs_linear_x = 0.2;
  config.planar_axis_epsilon = -1.0;
  EXPECT_THROW((void)VelocityAuthorityGate{config}, std::invalid_argument);
}

}  // namespace
