#include <gtest/gtest.h>

#include <chrono>
#include <string>

#include "embodied_agent_cpp/control_authority_tracker.hpp"

namespace
{

using namespace std::chrono_literals;
using embodied_agent_cpp::AuthorityUpdateDecision;
using embodied_agent_cpp::ControlAuthority;
using embodied_agent_cpp::ControlAuthoritySnapshot;
using embodied_agent_cpp::ControlAuthorityTracker;

ControlAuthoritySnapshot state(
  const ControlAuthority authority,
  const std::uint64_t sequence,
  const std::uint64_t epoch = 1U,
  const std::string & source = "")
{
  ControlAuthoritySnapshot output;
  output.authority = authority;
  output.estop_latched = authority == ControlAuthority::kEstop;
  output.manager_epoch = epoch;
  output.transition_sequence = sequence;
  output.pending_autonomy_revocation_sequence =
    authority == ControlAuthority::kAutonomy || sequence == 0U ?
    0U : sequence;
  output.autonomy_quiescence_acknowledged =
    authority != ControlAuthority::kAutonomy;
  output.active_source = source.empty() ?
    (authority == ControlAuthority::kAutonomy ? "autonomy" :
    authority == ControlAuthority::kKeyboard ? "keyboard_teleop" :
    authority == ControlAuthority::kEstop ? "safety_panel" : "") :
    source;
  output.reason =
    authority == ControlAuthority::kHold && sequence == 0U ?
    "initialized" : "test";
  return output;
}

TEST(ControlAuthorityTrackerTest, TracksFreshAuthorityAndTransitionsGeneration)
{
  ControlAuthorityTracker tracker(500ms);
  const auto start = ControlAuthorityTracker::TimePoint{};

  const auto initial = tracker.update(
    state(ControlAuthority::kHold, 0U), start);
  ASSERT_TRUE(initial.accepted);
  EXPECT_EQ(initial.decision, AuthorityUpdateDecision::kAcceptedInitial);
  EXPECT_EQ(tracker.generation(), 1U);
  EXPECT_TRUE(tracker.fresh(start + 500ms));
  EXPECT_FALSE(tracker.fresh_autonomy(start + 500ms));

  const auto autonomy = tracker.update(
    state(ControlAuthority::kAutonomy, 1U), start + 100ms);
  ASSERT_TRUE(autonomy.accepted);
  EXPECT_TRUE(autonomy.generation_changed);
  EXPECT_EQ(tracker.generation(), 2U);
  EXPECT_TRUE(tracker.fresh_autonomy(start + 200ms));
}

TEST(ControlAuthorityTrackerTest, RejectsOldAndConflictingSameSequenceStates)
{
  ControlAuthorityTracker tracker(500ms);
  const auto start = ControlAuthorityTracker::TimePoint{};
  ASSERT_TRUE(tracker.update(
      state(ControlAuthority::kAutonomy, 2U), start).accepted);

  EXPECT_FALSE(tracker.update(
      state(ControlAuthority::kHold, 1U), start + 10ms).accepted);
  EXPECT_FALSE(tracker.update(
      state(ControlAuthority::kKeyboard, 2U), start + 20ms).accepted);
  EXPECT_EQ(tracker.authority(), ControlAuthority::kAutonomy);
  EXPECT_EQ(tracker.generation(), 1U);
}

TEST(ControlAuthorityTrackerTest, LeaseGapPermanentlyRetiresTheCurrentEpoch)
{
  ControlAuthorityTracker tracker(500ms);
  const auto start = ControlAuthorityTracker::TimePoint{};
  const auto autonomy = state(ControlAuthority::kAutonomy, 1U);
  ASSERT_TRUE(tracker.update(autonomy, start).accepted);

  const auto heartbeat = tracker.update(autonomy, start + 501ms);
  EXPECT_FALSE(heartbeat.accepted);
  EXPECT_TRUE(heartbeat.lease_discontinuity);
  EXPECT_EQ(
    heartbeat.decision,
    AuthorityUpdateDecision::kRejectedLeaseDiscontinuity);
  EXPECT_FALSE(tracker.fresh_autonomy(start + 502ms));

  // 即使同一 epoch 随后提高 transition_sequence，也不能绕过失联边界。
  EXPECT_FALSE(tracker.update(
      state(ControlAuthority::kHold, 2U), start + 510ms).accepted);

  // 只有新 epoch 的安全 HOLD bootstrap 能恢复状态跟踪。
  const auto restarted = tracker.update(
    state(ControlAuthority::kHold, 0U, 2U), start + 520ms);
  ASSERT_TRUE(restarted.accepted);
  EXPECT_EQ(
    restarted.decision, AuthorityUpdateDecision::kAcceptedManagerRestart);
  EXPECT_EQ(tracker.authority(), ControlAuthority::kHold);
}

TEST(ControlAuthorityTrackerTest, ManagerRestartRequiresSafeBootstrapAndCannotRollBack)
{
  ControlAuthorityTracker tracker(500ms);
  const auto start = ControlAuthorityTracker::TimePoint{};
  ASSERT_TRUE(tracker.update(
      state(ControlAuthority::kAutonomy, 4U, 10U), start).accepted);

  EXPECT_FALSE(tracker.update(
      state(ControlAuthority::kAutonomy, 1U, 20U), start + 10ms).accepted);
  const auto restart = tracker.update(
    state(ControlAuthority::kHold, 0U, 20U), start + 20ms);
  ASSERT_TRUE(restart.accepted);
  EXPECT_EQ(
    restart.decision, AuthorityUpdateDecision::kAcceptedManagerRestart);
  EXPECT_EQ(tracker.authority(), ControlAuthority::kHold);

  EXPECT_FALSE(tracker.update(
      state(ControlAuthority::kHold, 0U, 10U), start + 30ms).accepted);
  EXPECT_EQ(tracker.authority(), ControlAuthority::kHold);
}

TEST(ControlAuthorityTrackerTest, RejectsMalformedPayloadWithoutRefreshingLease)
{
  ControlAuthorityTracker tracker(500ms);
  const auto start = ControlAuthorityTracker::TimePoint{};
  ASSERT_TRUE(tracker.update(
      state(ControlAuthority::kAutonomy, 1U), start).accepted);

  auto malformed = state(ControlAuthority::kEstop, 2U);
  malformed.estop_latched = false;
  EXPECT_FALSE(tracker.update(malformed, start + 400ms).accepted);
  EXPECT_FALSE(tracker.fresh(start + 501ms));
}

TEST(ControlAuthorityTrackerTest, QuiescenceAckMustUseANewTransitionSequence)
{
  ControlAuthorityTracker tracker(500ms);
  const auto start = ControlAuthorityTracker::TimePoint{};
  auto revoked = state(ControlAuthority::kHold, 2U);
  revoked.autonomy_quiescence_acknowledged = false;
  ASSERT_TRUE(tracker.update(revoked, start).accepted);

  // ACK 改变了安全语义；若 manager 没有递增序号，它只能被视为冲突心跳。
  auto conflicting_ack = revoked;
  conflicting_ack.autonomy_quiescence_acknowledged = true;
  EXPECT_FALSE(tracker.update(conflicting_ack, start + 10ms).accepted);

  conflicting_ack.transition_sequence = 3U;
  EXPECT_TRUE(tracker.update(conflicting_ack, start + 20ms).accepted);
  ASSERT_TRUE(tracker.snapshot());
  EXPECT_TRUE(tracker.snapshot()->autonomy_quiescence_acknowledged);
}

TEST(ControlAuthorityTrackerTest, RejectsAutonomyStateThatReusesAnOldAcknowledgement)
{
  ControlAuthorityTracker tracker(500ms);
  const auto start = ControlAuthorityTracker::TimePoint{};
  auto malformed = state(ControlAuthority::kAutonomy, 4U);
  malformed.pending_autonomy_revocation_sequence = 2U;
  malformed.autonomy_quiescence_acknowledged = true;

  EXPECT_FALSE(tracker.update(malformed, start).accepted);
  EXPECT_FALSE(tracker.observed());
}

}  // namespace
