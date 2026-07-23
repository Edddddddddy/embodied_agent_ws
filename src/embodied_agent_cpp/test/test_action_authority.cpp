#include <gtest/gtest.h>

#include "embodied_agent_cpp/action_authority.hpp"

namespace
{

using embodied_agent_cpp::ActionAuthorityDecision;
using embodied_agent_cpp::ControlAuthority;
using embodied_agent_cpp::evaluate_action_authority;

TEST(ActionAuthorityTest, DisabledGatePreservesLegacyDeployment)
{
  EXPECT_EQ(
    evaluate_action_authority(
      false, false, ControlAuthority::kHold, false),
    ActionAuthorityDecision::kAllow);
}

TEST(ActionAuthorityTest, OrdinaryActionsRequireFreshAutonomyAuthority)
{
  EXPECT_EQ(
    evaluate_action_authority(
      true, false, ControlAuthority::kAutonomy, false),
    ActionAuthorityDecision::kStateUnavailable);
  EXPECT_EQ(
    evaluate_action_authority(
      true, true, ControlAuthority::kHold, false),
    ActionAuthorityDecision::kHold);
  EXPECT_EQ(
    evaluate_action_authority(
      true, true, ControlAuthority::kKeyboard, false),
    ActionAuthorityDecision::kKeyboard);
  EXPECT_EQ(
    evaluate_action_authority(
      true, true, ControlAuthority::kEstop, false),
    ActionAuthorityDecision::kEmergencyStop);
  EXPECT_EQ(
    evaluate_action_authority(
      true, true, ControlAuthority::kAutonomy, false),
    ActionAuthorityDecision::kAllow);
}

TEST(ActionAuthorityTest, PriorityStopAlwaysUsesFailSafePath)
{
  for (const auto authority : {
      ControlAuthority::kHold,
      ControlAuthority::kAutonomy,
      ControlAuthority::kKeyboard,
      ControlAuthority::kEstop})
  {
    EXPECT_EQ(
      evaluate_action_authority(true, false, authority, true),
      ActionAuthorityDecision::kAllow);
    EXPECT_EQ(
      evaluate_action_authority(true, true, authority, true),
      ActionAuthorityDecision::kAllow);
  }
}

}  // namespace
