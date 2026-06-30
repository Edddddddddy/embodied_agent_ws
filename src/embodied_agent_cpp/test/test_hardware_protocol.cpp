#include <chrono>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "embodied_agent_cpp/hardware_protocol.hpp"

namespace embodied_agent_cpp
{
namespace
{

void expect_valid_crc(const std::vector<std::uint8_t> & frame)
{
  ASSERT_GE(frame.size(), 9U);
  const auto expected = static_cast<std::uint16_t>(frame[frame.size() - 2]) |
    static_cast<std::uint16_t>(frame.back()) << 8;
  EXPECT_EQ(expected, HardwareProtocol::crc16(frame.data(), frame.size() - 2));
}

TEST(HardwareProtocolTest, EncodesMoveAsVersionedCrcFrame)
{
  HardwareProtocol protocol;
  const auto result = protocol.encode(
    R"({"name":"move","arguments":{"linear_x":-0.2,"duration_s":1.0}})");
  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_EQ(result.action_name, "move");
  EXPECT_EQ(result.motion_duration, std::chrono::milliseconds(1000));
  ASSERT_EQ(result.frame.size(), 13U);
  EXPECT_EQ(result.frame[0], 0xaa);
  EXPECT_EQ(result.frame[1], 0x55);
  EXPECT_EQ(result.frame[2], 0x01);
  EXPECT_EQ(result.frame[3], static_cast<std::uint8_t>(HardwareOpcode::move));
  EXPECT_EQ(result.frame[4], 4);
  EXPECT_EQ(result.frame[7], 0x38);
  EXPECT_EQ(result.frame[8], 0xff);
  EXPECT_EQ(result.frame[9], 0xe8);
  EXPECT_EQ(result.frame[10], 0x03);
  expect_valid_crc(result.frame);
}

TEST(HardwareProtocolTest, SequenceIncrementsAndLedUsesRgbPayload)
{
  HardwareProtocol protocol;
  const auto stop = protocol.encode_stop();
  const auto led = protocol.encode(
    R"({"name":"set_led","arguments":{"color":"yellow"}})");
  ASSERT_TRUE(led.valid);
  EXPECT_EQ(led.sequence, stop.sequence + 1);
  ASSERT_EQ(led.frame.size(), 12U);
  EXPECT_EQ(led.frame[7], 255);
  EXPECT_EQ(led.frame[8], 255);
  EXPECT_EQ(led.frame[9], 0);
  expect_valid_crc(led.frame);
}

TEST(HardwareProtocolTest, RejectsMalformedOrUnsupportedCommands)
{
  HardwareProtocol protocol;
  EXPECT_FALSE(protocol.encode("not-json").valid);
  EXPECT_FALSE(protocol.encode(
      R"({"name":"set_led","arguments":{"color":"purple"}})").valid);
  EXPECT_FALSE(protocol.encode(
      R"({"name":"fly","arguments":{}})").valid);
}

TEST(MotionWatchdogTest, ExpiresExactlyOnce)
{
  MotionWatchdog watchdog;
  const auto start = MotionWatchdog::Clock::time_point{};
  watchdog.arm(std::chrono::milliseconds(500), start);
  EXPECT_TRUE(watchdog.armed());
  EXPECT_FALSE(watchdog.expired(start + std::chrono::milliseconds(499)));
  EXPECT_TRUE(watchdog.expired(start + std::chrono::milliseconds(500)));
  EXPECT_FALSE(watchdog.expired(start + std::chrono::milliseconds(600)));
}

}  // namespace
}  // namespace embodied_agent_cpp
