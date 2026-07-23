#include <chrono>
#include <stdexcept>

#include "gtest/gtest.h"

#include "embodied_agent_cpp/keyboard_input.hpp"

namespace
{

using namespace std::chrono_literals;
using embodied_agent_cpp::KeyboardInput;
using embodied_agent_cpp::KeyboardInputConfig;
using embodied_agent_cpp::KeyboardIntent;

TEST(KeyboardInputTest, MapsMotionKeysToRosVelocityConvention)
{
  KeyboardInput input;
  const auto start = KeyboardInput::TimePoint{};

  auto update = input.apply_key('W', start);
  EXPECT_EQ(update.intent, KeyboardIntent::kMotion);
  EXPECT_DOUBLE_EQ(update.linear_x, 0.2);
  EXPECT_DOUBLE_EQ(update.angular_z, 0.0);

  update = input.apply_key('s', start + 10ms);
  EXPECT_DOUBLE_EQ(update.linear_x, -0.2);
  EXPECT_DOUBLE_EQ(update.angular_z, 0.0);

  update = input.apply_key('a', start + 20ms);
  EXPECT_DOUBLE_EQ(update.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(update.angular_z, 0.8);

  update = input.apply_key('D', start + 30ms);
  EXPECT_DOUBLE_EQ(update.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(update.angular_z, -0.8);
}

TEST(KeyboardInputTest, ClampsConfiguredSpeedsAtSafetyLimits)
{
  KeyboardInputConfig config;
  config.linear_speed = 2.0;
  config.angular_speed = 5.0;
  config.max_linear_speed = 0.3;
  config.max_angular_speed = 1.2;
  KeyboardInput input(config);

  auto update = input.apply_key('w', KeyboardInput::TimePoint{});
  EXPECT_DOUBLE_EQ(update.linear_x, 0.3);
  update = input.apply_key('a', KeyboardInput::TimePoint{});
  EXPECT_DOUBLE_EQ(update.angular_z, 1.2);
}

TEST(KeyboardInputTest, DeadmanStopsMotionAfterSixHundredMilliseconds)
{
  KeyboardInput input;
  const auto start = KeyboardInput::TimePoint{};
  input.apply_key('w', start);

  auto update = input.update_deadman(start + 599ms);
  EXPECT_TRUE(input.motion_active());
  EXPECT_DOUBLE_EQ(update.linear_x, 0.2);
  EXPECT_EQ(update.intent, KeyboardIntent::kNone);

  update = input.update_deadman(start + 600ms);
  EXPECT_FALSE(input.motion_active());
  EXPECT_DOUBLE_EQ(update.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(update.angular_z, 0.0);
  EXPECT_EQ(update.intent, KeyboardIntent::kDeadmanStop);

  // 超时事件只发一次，避免服务端被 20 Hz 的重复 stop 淹没。
  update = input.update_deadman(start + 700ms);
  EXPECT_EQ(update.intent, KeyboardIntent::kNone);
}

TEST(KeyboardInputTest, RepeatedMotionInputRefreshesDeadman)
{
  KeyboardInput input;
  const auto start = KeyboardInput::TimePoint{};
  input.apply_key('w', start);
  input.apply_key('w', start + 250ms);

  EXPECT_TRUE(input.motion_active());
  EXPECT_EQ(input.update_deadman(start + 500ms).intent, KeyboardIntent::kNone);
  EXPECT_EQ(input.update_deadman(start + 850ms).intent, KeyboardIntent::kDeadmanStop);
}

TEST(KeyboardInputTest, EmergencyStopIsLatchedUntilExplicitReset)
{
  KeyboardInput input;
  const auto start = KeyboardInput::TimePoint{};
  input.apply_key('w', start);

  auto update = input.apply_key('x', start + 10ms);
  EXPECT_EQ(update.intent, KeyboardIntent::kEmergencyStop);
  EXPECT_TRUE(update.emergency_stop_latched);
  EXPECT_DOUBLE_EQ(update.linear_x, 0.0);

  update = input.apply_key('w', start + 20ms);
  EXPECT_EQ(update.intent, KeyboardIntent::kBlockedByEmergencyStop);
  EXPECT_TRUE(update.emergency_stop_latched);
  EXPECT_DOUBLE_EQ(update.linear_x, 0.0);

  update = input.apply_key('r', start + 30ms);
  EXPECT_EQ(update.intent, KeyboardIntent::kResetAndResume);
  EXPECT_FALSE(update.emergency_stop_latched);

  update = input.apply_key('w', start + 40ms);
  EXPECT_EQ(update.intent, KeyboardIntent::kMotion);
  EXPECT_DOUBLE_EQ(update.linear_x, 0.2);
}

TEST(KeyboardInputTest, SpaceAndQuitAlwaysReturnZeroVelocity)
{
  KeyboardInput input;
  const auto start = KeyboardInput::TimePoint{};
  input.apply_key('a', start);

  auto update = input.apply_key(' ', start + 10ms);
  EXPECT_EQ(update.intent, KeyboardIntent::kSoftStop);
  EXPECT_DOUBLE_EQ(update.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(update.angular_z, 0.0);

  input.apply_key('w', start + 20ms);
  update = input.apply_key('q', start + 30ms);
  EXPECT_EQ(update.intent, KeyboardIntent::kSafeExit);
  EXPECT_DOUBLE_EQ(update.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(update.angular_z, 0.0);
}

TEST(KeyboardInputTest, RejectsInvalidSafetyConfiguration)
{
  KeyboardInputConfig config;
  config.max_linear_speed = 0.0;
  EXPECT_THROW((void)KeyboardInput{config}, std::invalid_argument);

  config = KeyboardInputConfig{};
  config.deadman_timeout = 0ms;
  EXPECT_THROW((void)KeyboardInput{config}, std::invalid_argument);
}

}  // namespace
