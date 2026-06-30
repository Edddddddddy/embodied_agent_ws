#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "embodied_agent_cpp/hardware_transport.hpp"

namespace embodied_agent_cpp
{
namespace
{

TEST(UartTransportTest, WritesExactFrameThroughConfiguredPseudoTerminal)
{
  const int master = posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
  ASSERT_GE(master, 0);
  ASSERT_EQ(grantpt(master), 0);
  ASSERT_EQ(unlockpt(master), 0);
  const char * slave_name = ptsname(master);
  ASSERT_NE(slave_name, nullptr);

  auto transport = make_uart_transport(slave_name, 115200);
  std::string error;
  ASSERT_TRUE(transport->open(error)) << error;
  const std::vector<std::uint8_t> frame{0xaa, 0x55, 0x01, 0x01, 0x00, 0x01, 0x00, 0x7c, 0x3d};
  ASSERT_TRUE(transport->send(frame, error)) << error;

  pollfd event{master, POLLIN, 0};
  ASSERT_GT(poll(&event, 1, 500), 0);
  std::vector<std::uint8_t> received(frame.size());
  const auto count = read(master, received.data(), received.size());
  close(master);
  ASSERT_EQ(count, static_cast<ssize_t>(frame.size()));
  EXPECT_EQ(received, frame);
}

TEST(UartTransportTest, RejectsUnsupportedBaudRate)
{
  auto transport = make_uart_transport("/dev/null", 12345);
  std::string error;
  EXPECT_FALSE(transport->open(error));
  EXPECT_EQ(error, "unsupported UART baud rate");
}

}  // namespace
}  // namespace embodied_agent_cpp
