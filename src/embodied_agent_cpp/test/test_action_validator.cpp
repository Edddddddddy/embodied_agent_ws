#include <string>

#include "gtest/gtest.h"

#include "embodied_agent_cpp/action_validator.hpp"

TEST(ActionValidatorTest, ClampsMoveCommand)
{
  embodied_agent_cpp::ActionValidator validator;
  const auto result = validator.validate(
    R"({"name":"move","arguments":{"linear_x":9.0,"duration_s":20.0}})");
  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_DOUBLE_EQ(result.command["arguments"]["linear_x"], 0.5);
  EXPECT_DOUBLE_EQ(result.command["arguments"]["duration_s"], 10.0);
}

TEST(ActionValidatorTest, NormalizesArcAsCurvedMove)
{
  embodied_agent_cpp::ActionValidator validator;
  const auto result = validator.validate(
    R"({"name":"arc","arguments":{"linear_x":9.0,"angular_z":9.0,"duration_s":20.0}})");

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_EQ(result.command["name"], "move");
  EXPECT_DOUBLE_EQ(result.command["arguments"]["linear_x"], 0.5);
  EXPECT_DOUBLE_EQ(result.command["arguments"]["angular_z"], 1.5);
  EXPECT_DOUBLE_EQ(result.command["arguments"]["duration_s"], 10.0);
}

TEST(ActionValidatorTest, RejectsUnknownAction)
{
  embodied_agent_cpp::ActionValidator validator;
  const auto result = validator.validate(
    R"({"name":"launch_missile","arguments":{}})");
  EXPECT_FALSE(result.valid);
}

TEST(ActionValidatorTest, RejectsMissingAndExtraArguments)
{
  embodied_agent_cpp::ActionValidator validator;
  EXPECT_FALSE(validator.validate(
      R"({"name":"move","arguments":{"linear_x":0.2}})").valid);
  EXPECT_FALSE(validator.validate(
      R"({"name":"stop","arguments":{"duration_s":1.0}})").valid);
}

TEST(ActionValidatorTest, ValidatesOptionalRequestId)
{
  embodied_agent_cpp::ActionValidator validator;
  EXPECT_TRUE(validator.validate(
      R"({"name":"stop","request_id":"agent-action-1","arguments":{}})").valid);
  EXPECT_FALSE(validator.validate(
      R"({"name":"stop","request_id":42,"arguments":{}})").valid);
}

TEST(ActionValidatorTest, ValidatesLedColorType)
{
  embodied_agent_cpp::ActionValidator validator;
  EXPECT_TRUE(validator.validate(
      R"({"name":"set_led","arguments":{"color":"blue"}})").valid);
  EXPECT_FALSE(validator.validate(
      R"({"name":"set_led","arguments":{"color":12}})").valid);
}

TEST(ActionValidatorTest, RejectsUnsupportedLedColor)
{
  embodied_agent_cpp::ActionValidator validator;
  const auto result = validator.validate(
    R"({"name":"set_led","arguments":{"color":"purple"}})");
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.error, "unsupported LED color");
}

TEST(ActionValidatorTest, ValidatesSimulationControlMode)
{
  embodied_agent_cpp::ActionValidator validator;
  EXPECT_TRUE(validator.validate(
      R"({"name":"set_mode","arguments":{"mode":"obstacle_avoidance"}})").valid);
  const auto result = validator.validate(
    R"({"name":"set_mode","arguments":{"mode":"unsafe_racing"}})");
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.error, "unsupported control mode");
}

TEST(ActionValidatorTest, ValidatesNavigationTargetsAndWaypointLoops)
{
  embodied_agent_cpp::ActionValidator validator;
  const auto target = validator.validate(
    R"({"name":"navigate_to","arguments":{"target":"door"}})");
  ASSERT_TRUE(target.valid) << target.error;

  const auto patrol = validator.validate(
    R"({"name":"patrol","arguments":{"waypoints":["door","desk","home"],"number_of_loops":9}})");
  ASSERT_TRUE(patrol.valid) << patrol.error;
  EXPECT_EQ(patrol.command["name"], "follow_waypoints");
  EXPECT_EQ(patrol.command["arguments"]["number_of_loops"], 3);

  const auto unsupported = validator.validate(
    R"({"name":"navigate_to","arguments":{"target":"server_room"}})");
  EXPECT_FALSE(unsupported.valid);
  EXPECT_EQ(unsupported.error, "unsupported navigation target");
}

TEST(ActionValidatorTest, ValidatesCancelNavigation)
{
  embodied_agent_cpp::ActionValidator validator;
  EXPECT_TRUE(validator.validate(
      R"({"name":"cancel_navigation","arguments":{}})").valid);
  EXPECT_FALSE(validator.validate(
      R"({"name":"cancel_navigation","arguments":{"target":"door"}})").valid);
}
