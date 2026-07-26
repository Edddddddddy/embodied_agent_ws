#include <gtest/gtest.h>

#include <string>

#include "embodied_agent_cpp/control_authority.hpp"

namespace
{

using embodied_agent_cpp::ControlAuthority;
using embodied_agent_cpp::AutonomyQuiescenceAcknowledgement;
using embodied_agent_cpp::ControlAuthorityCommand;
using embodied_agent_cpp::ControlAuthorityMachine;
using embodied_agent_cpp::ControlAuthorityRequest;

ControlAuthorityRequest request(
  const ControlAuthorityCommand command,
  const std::string & requester = "keyboard")
{
  ControlAuthorityRequest output;
  output.command = command;
  output.requester = requester;
  return output;
}

TEST(ControlAuthorityTest, KeyboardTakeoverStopsAutonomyAndDeadmanEndsInHold)
{
  ControlAuthorityMachine machine(1U, true);

  const auto resumed = machine.request(request(
      ControlAuthorityCommand::kResumeAutonomy, "operator"));
  ASSERT_TRUE(resumed.accepted);
  EXPECT_TRUE(resumed.changed);
  EXPECT_FALSE(resumed.stop_requested);
  EXPECT_EQ(resumed.state.authority, ControlAuthority::kAutonomy);
  EXPECT_EQ(resumed.state.transition_sequence, 1U);

  const auto takeover = machine.request(request(
      ControlAuthorityCommand::kTakeKeyboard));
  ASSERT_TRUE(takeover.accepted);
  EXPECT_TRUE(takeover.changed);
  EXPECT_TRUE(takeover.stop_requested);
  EXPECT_EQ(takeover.state.authority, ControlAuthority::kKeyboard);
  EXPECT_EQ(takeover.state.active_source, "keyboard");
  EXPECT_EQ(takeover.state.transition_sequence, 2U);

  const auto released = machine.request(request(
      ControlAuthorityCommand::kReleaseKeyboard));
  ASSERT_TRUE(released.accepted);
  EXPECT_TRUE(released.changed);
  EXPECT_TRUE(released.stop_requested);
  EXPECT_EQ(released.state.authority, ControlAuthority::kHold);
  EXPECT_TRUE(released.state.active_source.empty());
  EXPECT_EQ(released.state.transition_sequence, 3U);
}

TEST(ControlAuthorityTest, HoldRequiresExplicitResumeBeforeAutonomyReturns)
{
  ControlAuthorityMachine machine(1U, true);
  machine.request(request(ControlAuthorityCommand::kTakeKeyboard));
  machine.request(request(ControlAuthorityCommand::kReleaseKeyboard));

  EXPECT_EQ(machine.snapshot().authority, ControlAuthority::kHold);
  const auto resumed = machine.request(request(
      ControlAuthorityCommand::kResumeAutonomy, "operator"));

  ASSERT_TRUE(resumed.accepted);
  EXPECT_TRUE(resumed.changed);
  EXPECT_FALSE(resumed.stop_requested);
  EXPECT_EQ(resumed.state.authority, ControlAuthority::kAutonomy);
}

TEST(ControlAuthorityTest, EmergencyStopIsLatchedAndResetStillLeavesHold)
{
  ControlAuthorityMachine machine(1U, true);
  machine.request(request(ControlAuthorityCommand::kResumeAutonomy, "operator"));

  const auto stopped = machine.request(request(
      ControlAuthorityCommand::kEmergencyStop, "safety_panel"));
  ASSERT_TRUE(stopped.accepted);
  EXPECT_TRUE(stopped.changed);
  EXPECT_TRUE(stopped.stop_requested);
  EXPECT_EQ(stopped.state.authority, ControlAuthority::kEstop);
  EXPECT_TRUE(stopped.state.estop_latched);

  const auto blocked_resume = machine.request(request(
      ControlAuthorityCommand::kResumeAutonomy, "operator"));
  EXPECT_FALSE(blocked_resume.accepted);
  EXPECT_EQ(blocked_resume.state.authority, ControlAuthority::kEstop);

  const auto reset = machine.request(request(
      ControlAuthorityCommand::kResetEmergencyStop, "safety_panel"));
  ASSERT_TRUE(reset.accepted);
  EXPECT_TRUE(reset.changed);
  EXPECT_FALSE(reset.stop_requested);
  EXPECT_EQ(reset.state.authority, ControlAuthority::kHold);
  EXPECT_FALSE(reset.state.estop_latched);

  AutonomyQuiescenceAcknowledgement acknowledgement;
  acknowledgement.manager_epoch = reset.state.manager_epoch;
  acknowledgement.autonomy_revocation_sequence =
    reset.state.pending_autonomy_revocation_sequence;
  acknowledgement.requester = "mission_orchestrator";
  ASSERT_TRUE(
    machine.acknowledge_autonomy_quiescence(acknowledgement).accepted);

  const auto resumed = machine.request(request(
      ControlAuthorityCommand::kResumeAutonomy, "operator"));
  EXPECT_TRUE(resumed.accepted);
  EXPECT_EQ(resumed.state.authority, ControlAuthority::kAutonomy);
}

TEST(ControlAuthorityTest, RepeatedSafetyRequestsDoNotRepeatStopIntent)
{
  ControlAuthorityMachine machine(1U, true);
  machine.request(request(ControlAuthorityCommand::kResumeAutonomy, "operator"));

  const auto first_takeover = machine.request(request(
      ControlAuthorityCommand::kTakeKeyboard));
  const auto repeated_takeover = machine.request(request(
      ControlAuthorityCommand::kTakeKeyboard));
  EXPECT_TRUE(first_takeover.stop_requested);
  EXPECT_FALSE(repeated_takeover.changed);
  EXPECT_FALSE(repeated_takeover.stop_requested);
  EXPECT_EQ(repeated_takeover.state.transition_sequence, 2U);

  const auto first_estop = machine.request(request(
      ControlAuthorityCommand::kEmergencyStop, "safety_panel"));
  const auto repeated_estop = machine.request(request(
      ControlAuthorityCommand::kEmergencyStop, "safety_panel"));
  EXPECT_TRUE(first_estop.stop_requested);
  EXPECT_FALSE(repeated_estop.changed);
  EXPECT_FALSE(repeated_estop.stop_requested);
  EXPECT_EQ(repeated_estop.state.transition_sequence, 3U);
}

TEST(ControlAuthorityTest, KeyboardOwnershipAndInvalidTransitionsFailClosed)
{
  ControlAuthorityMachine machine;
  machine.request(request(ControlAuthorityCommand::kTakeKeyboard, "terminal-a"));

  const auto foreign_release = machine.request(request(
      ControlAuthorityCommand::kReleaseKeyboard, "terminal-b"));
  EXPECT_FALSE(foreign_release.accepted);
  EXPECT_EQ(foreign_release.state.authority, ControlAuthority::kKeyboard);
  EXPECT_EQ(foreign_release.state.transition_sequence, 1U);

  const auto direct_resume = machine.request(request(
      ControlAuthorityCommand::kResumeAutonomy, "operator"));
  EXPECT_FALSE(direct_resume.accepted);
  EXPECT_EQ(direct_resume.state.transition_sequence, 1U);

  const auto anonymous_estop = machine.request(request(
      ControlAuthorityCommand::kEmergencyStop, ""));
  EXPECT_FALSE(anonymous_estop.accepted);
  EXPECT_EQ(anonymous_estop.state.transition_sequence, 1U);
}

TEST(ControlAuthorityTest, ExplicitHoldStopsEitherActiveMotionOwnerOnce)
{
  ControlAuthorityMachine machine(1U, true);
  machine.request(request(ControlAuthorityCommand::kResumeAutonomy, "operator"));

  const auto hold = machine.request(request(
      ControlAuthorityCommand::kEnterHold, "operator"));
  ASSERT_TRUE(hold.accepted);
  EXPECT_TRUE(hold.stop_requested);
  EXPECT_EQ(hold.state.authority, ControlAuthority::kHold);

  const auto repeated_hold = machine.request(request(
      ControlAuthorityCommand::kEnterHold, "operator"));
  EXPECT_TRUE(repeated_hold.accepted);
  EXPECT_FALSE(repeated_hold.changed);
  EXPECT_FALSE(repeated_hold.stop_requested);
  EXPECT_EQ(repeated_hold.state.transition_sequence, hold.state.transition_sequence);
}

TEST(ControlAuthorityTest, ResumeWaitsForExactAutonomyQuiescenceAcknowledgement)
{
  ControlAuthorityMachine machine(42U, true);
  ASSERT_TRUE(machine.request(request(
      ControlAuthorityCommand::kResumeAutonomy, "operator")).accepted);

  const auto hold = machine.request(request(
      ControlAuthorityCommand::kEnterHold, "operator"));
  ASSERT_TRUE(hold.accepted);
  ASSERT_EQ(hold.state.pending_autonomy_revocation_sequence, 2U);
  EXPECT_FALSE(hold.state.autonomy_quiescence_acknowledged);

  const auto blocked = machine.request(request(
      ControlAuthorityCommand::kResumeAutonomy, "operator"));
  EXPECT_FALSE(blocked.accepted);
  EXPECT_EQ(blocked.message, "autonomy_quiescence_not_acknowledged");

  AutonomyQuiescenceAcknowledgement acknowledgement;
  acknowledgement.manager_epoch = 42U;
  acknowledgement.autonomy_revocation_sequence = 2U;
  acknowledgement.requester = "mission_orchestrator";
  acknowledgement.detail = "Nav2 goal reached terminal CANCELED and velocity is zero";
  const auto acknowledged =
    machine.acknowledge_autonomy_quiescence(acknowledgement);
  ASSERT_TRUE(acknowledged.accepted);
  EXPECT_TRUE(acknowledged.changed);
  EXPECT_TRUE(acknowledged.state.autonomy_quiescence_acknowledged);
  EXPECT_EQ(acknowledged.state.pending_autonomy_revocation_sequence, 2U);

  const auto resumed = machine.request(request(
      ControlAuthorityCommand::kResumeAutonomy, "operator"));
  ASSERT_TRUE(resumed.accepted);
  EXPECT_EQ(resumed.state.authority, ControlAuthority::kAutonomy);
  // ACK 只授权本次恢复，不能成为后续重启/切权的永久通行证。
  EXPECT_EQ(resumed.state.pending_autonomy_revocation_sequence, 0U);
  EXPECT_FALSE(resumed.state.autonomy_quiescence_acknowledged);
}

TEST(ControlAuthorityTest, RejectsStaleEpochAndRevocationAcknowledgements)
{
  ControlAuthorityMachine machine(42U, true);
  machine.request(request(ControlAuthorityCommand::kResumeAutonomy, "operator"));
  const auto hold = machine.request(request(
      ControlAuthorityCommand::kEnterHold, "operator"));

  AutonomyQuiescenceAcknowledgement acknowledgement;
  acknowledgement.manager_epoch = 41U;
  acknowledgement.autonomy_revocation_sequence =
    hold.state.pending_autonomy_revocation_sequence;
  acknowledgement.requester = "mission_orchestrator";
  const auto stale_epoch =
    machine.acknowledge_autonomy_quiescence(acknowledgement);
  EXPECT_FALSE(stale_epoch.accepted);
  EXPECT_EQ(stale_epoch.message, "manager_epoch_mismatch");

  acknowledgement.manager_epoch = 42U;
  --acknowledgement.autonomy_revocation_sequence;
  const auto stale_sequence =
    machine.acknowledge_autonomy_quiescence(acknowledgement);
  EXPECT_FALSE(stale_sequence.accepted);
  EXPECT_EQ(
    stale_sequence.message, "autonomy_revocation_sequence_mismatch");
  EXPECT_FALSE(machine.snapshot().autonomy_quiescence_acknowledged);
}

TEST(ControlAuthorityTest, AcknowledgementSurvivesHoldKeyboardAndEstopUntilResumeConsumesIt)
{
  ControlAuthorityMachine machine(7U, true);
  machine.request(request(ControlAuthorityCommand::kResumeAutonomy, "operator"));
  const auto keyboard = machine.request(request(
      ControlAuthorityCommand::kTakeKeyboard, "keyboard"));
  const auto revocation_sequence =
    keyboard.state.pending_autonomy_revocation_sequence;

  AutonomyQuiescenceAcknowledgement acknowledgement;
  acknowledgement.manager_epoch = 7U;
  acknowledgement.autonomy_revocation_sequence = revocation_sequence;
  acknowledgement.requester = "mission_orchestrator";
  ASSERT_TRUE(
    machine.acknowledge_autonomy_quiescence(acknowledgement).accepted);

  machine.request(request(ControlAuthorityCommand::kEmergencyStop, "safety_panel"));
  const auto reset = machine.request(request(
      ControlAuthorityCommand::kResetEmergencyStop, "safety_panel"));
  EXPECT_EQ(
    reset.state.pending_autonomy_revocation_sequence, revocation_sequence);
  EXPECT_TRUE(reset.state.autonomy_quiescence_acknowledged);

  const auto resumed = machine.request(request(
      ControlAuthorityCommand::kResumeAutonomy, "operator"));
  ASSERT_TRUE(resumed.accepted);
  EXPECT_EQ(resumed.state.pending_autonomy_revocation_sequence, 0U);
  EXPECT_FALSE(resumed.state.autonomy_quiescence_acknowledged);
}

TEST(ControlAuthorityTest, StrictBootstrapRequiresSequenceZeroAcknowledgement)
{
  // 默认构造也必须 fail-closed，防止未来调用者遗漏节点参数后重新放宽启动语义。
  ControlAuthorityMachine machine(9U);
  const auto blocked = machine.request(request(
      ControlAuthorityCommand::kResumeAutonomy, "operator"));
  EXPECT_FALSE(blocked.accepted);
  EXPECT_EQ(blocked.message, "autonomy_quiescence_not_acknowledged");

  AutonomyQuiescenceAcknowledgement acknowledgement;
  acknowledgement.manager_epoch = 9U;
  acknowledgement.autonomy_revocation_sequence = 0U;
  acknowledgement.requester = "system_orchestrator";
  const auto acknowledged =
    machine.acknowledge_autonomy_quiescence(acknowledgement);
  ASSERT_TRUE(acknowledged.accepted);
  EXPECT_TRUE(acknowledged.state.autonomy_quiescence_acknowledged);

  EXPECT_TRUE(machine.request(request(
      ControlAuthorityCommand::kResumeAutonomy, "operator")).accepted);
}

}  // namespace
