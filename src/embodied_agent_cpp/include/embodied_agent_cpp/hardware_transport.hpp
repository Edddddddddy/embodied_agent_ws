#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace embodied_agent_cpp
{

class HardwareTransport
{
public:
  virtual ~HardwareTransport() = default;
  virtual bool open(std::string & error) = 0;
  virtual bool send(const std::vector<std::uint8_t> & frame, std::string & error) = 0;
  virtual std::string name() const = 0;
};

std::unique_ptr<HardwareTransport> make_mock_transport();
std::unique_ptr<HardwareTransport> make_uart_transport(
  const std::string & device, int baud_rate);
std::unique_ptr<HardwareTransport> make_spi_transport(
  const std::string & device, std::uint32_t speed_hz);

}  // namespace embodied_agent_cpp
