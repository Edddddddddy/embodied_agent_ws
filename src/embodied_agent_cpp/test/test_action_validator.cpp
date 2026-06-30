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

TEST(ActionValidatorTest, ValidatesLedColorType)
{
  embodied_agent_cpp::ActionValidator validator;
  EXPECT_TRUE(validator.validate(
      R"({"name":"set_led","arguments":{"color":"blue"}})").valid);
  EXPECT_FALSE(validator.validate(
      R"({"name":"set_led","arguments":{"color":12}})").valid);
}

