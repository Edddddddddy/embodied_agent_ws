#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace embodied_agent_cpp
{

enum class HardwareOpcode : std::uint8_t
{
  stop = 0x01,
  move = 0x02,
  turn = 0x03,
  wave = 0x04,
  set_led = 0x05,
};

struct EncodedHardwareCommand
{
  bool valid{false};
  std::string error;
  std::string action_name;
  std::uint16_t sequence{0};
  std::chrono::milliseconds motion_duration{0};
  std::vector<std::uint8_t> frame;
};

class HardwareProtocol
{
public:
  EncodedHardwareCommand encode(const std::string & json_command);
  EncodedHardwareCommand encode_stop();
  static std::uint16_t crc16(const std::uint8_t * data, std::size_t size);

private:
  EncodedHardwareCommand frame(
    HardwareOpcode opcode, const std::string & action_name,
    std::vector<std::uint8_t> payload, std::chrono::milliseconds duration = {});
  std::uint16_t next_sequence_{1};
};

class MotionWatchdog
{
public:
  using Clock = std::chrono::steady_clock;
  void arm(std::chrono::milliseconds duration, Clock::time_point now);
  void disarm();
  bool expired(Clock::time_point now);
  bool armed() const {return deadline_.has_value();}

private:
  std::optional<Clock::time_point> deadline_;
};

}  // namespace embodied_agent_cpp
