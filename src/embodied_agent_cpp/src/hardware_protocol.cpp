#include "embodied_agent_cpp/hardware_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

#include <nlohmann/json.hpp>

namespace embodied_agent_cpp
{
namespace
{

void append_i16(std::vector<std::uint8_t> & output, std::int16_t value)
{
  const auto raw = static_cast<std::uint16_t>(value);
  output.push_back(static_cast<std::uint8_t>(raw & 0xff));
  output.push_back(static_cast<std::uint8_t>((raw >> 8) & 0xff));
}

void append_u16(std::vector<std::uint8_t> & output, std::uint16_t value)
{
  output.push_back(static_cast<std::uint8_t>(value & 0xff));
  output.push_back(static_cast<std::uint8_t>((value >> 8) & 0xff));
}

EncodedHardwareCommand invalid_command(std::string error)
{
  EncodedHardwareCommand result;
  result.error = std::move(error);
  return result;
}

}  // namespace

EncodedHardwareCommand HardwareProtocol::encode(const std::string & serialized)
{
  nlohmann::json command;
  try {
    command = nlohmann::json::parse(serialized);
    const auto name = command.at("name").get<std::string>();
    const auto & arguments = command.at("arguments");
    if (name == "stop") {
      return encode_stop();
    }
    if (name == "move") {
      std::vector<std::uint8_t> payload;
      const auto velocity = static_cast<std::int16_t>(
        std::lround(arguments.at("linear_x").get<double>() * 1000.0));
      const auto duration = static_cast<std::uint16_t>(
        std::lround(arguments.at("duration_s").get<double>() * 1000.0));
      append_i16(payload, velocity);
      append_u16(payload, duration);
      return frame(HardwareOpcode::move, name, std::move(payload), std::chrono::milliseconds(duration));
    }
    if (name == "turn") {
      std::vector<std::uint8_t> payload;
      const auto angular = static_cast<std::int16_t>(
        std::lround(arguments.at("angular_z").get<double>() * 1000.0));
      const auto duration = static_cast<std::uint16_t>(
        std::lround(arguments.at("duration_s").get<double>() * 1000.0));
      append_i16(payload, angular);
      append_u16(payload, duration);
      return frame(HardwareOpcode::turn, name, std::move(payload), std::chrono::milliseconds(duration));
    }
    if (name == "wave") {
      return frame(
        HardwareOpcode::wave, name,
        {static_cast<std::uint8_t>(arguments.at("count").get<int>())});
    }
    if (name == "set_led") {
      static const std::map<std::string, std::vector<std::uint8_t>> colors{
        {"off", {0, 0, 0}}, {"red", {255, 0, 0}}, {"green", {0, 255, 0}},
        {"blue", {0, 0, 255}}, {"yellow", {255, 255, 0}}, {"white", {255, 255, 255}},
      };
      const auto color = arguments.at("color").get<std::string>();
      const auto found = colors.find(color);
      if (found == colors.end()) {
        return invalid_command("unsupported LED color: " + color);
      }
      return frame(HardwareOpcode::set_led, name, found->second);
    }
    return invalid_command("unsupported hardware action: " + name);
  } catch (const nlohmann::json::exception & error) {
    return invalid_command(std::string("invalid hardware command: ") + error.what());
  }
}

EncodedHardwareCommand HardwareProtocol::encode_stop()
{
  return frame(HardwareOpcode::stop, "stop", {});
}

EncodedHardwareCommand HardwareProtocol::frame(
  HardwareOpcode opcode, const std::string & action_name,
  std::vector<std::uint8_t> payload, std::chrono::milliseconds duration)
{
  EncodedHardwareCommand result;
  result.valid = true;
  result.action_name = action_name;
  result.sequence = next_sequence_++;
  result.motion_duration = duration;
  result.frame = {
    0xaa, 0x55, 0x01, static_cast<std::uint8_t>(opcode),
    static_cast<std::uint8_t>(payload.size()),
    static_cast<std::uint8_t>(result.sequence & 0xff),
    static_cast<std::uint8_t>((result.sequence >> 8) & 0xff),
  };
  result.frame.insert(result.frame.end(), payload.begin(), payload.end());
  const auto checksum = crc16(result.frame.data(), result.frame.size());
  append_u16(result.frame, checksum);
  return result;
}

std::uint16_t HardwareProtocol::crc16(const std::uint8_t * data, std::size_t size)
{
  std::uint16_t crc = 0xffff;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= static_cast<std::uint16_t>(data[index]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021) :
        static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

void MotionWatchdog::arm(std::chrono::milliseconds duration, Clock::time_point now)
{
  deadline_ = now + duration;
}

void MotionWatchdog::disarm()
{
  deadline_.reset();
}

bool MotionWatchdog::expired(Clock::time_point now)
{
  if (!deadline_ || now < *deadline_) {
    return false;
  }
  deadline_.reset();
  return true;
}

}  // namespace embodied_agent_cpp
