#include "embodied_agent_cpp/hardware_transport.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include <utility>

namespace embodied_agent_cpp
{
namespace
{

class FileDescriptor
{
public:
  ~FileDescriptor() {if (value_ >= 0) {::close(value_);}}
  int get() const {return value_;}
  void reset(int value) {if (value_ >= 0) {::close(value_);} value_ = value;}

private:
  int value_{-1};
};

class MockTransport final : public HardwareTransport
{
public:
  bool open(std::string &) override {return true;}
  bool send(const std::vector<std::uint8_t> & frame, std::string &) override
  {
    last_frame_ = frame;
    return true;
  }
  std::string name() const override {return "mock";}

private:
  std::vector<std::uint8_t> last_frame_;
};

speed_t baud_constant(int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return 0;
  }
}

class UartTransport final : public HardwareTransport
{
public:
  UartTransport(std::string device, int baud_rate)
  : device_(std::move(device)), baud_rate_(baud_rate) {}

  bool open(std::string & error) override
  {
    const auto baud = baud_constant(baud_rate_);
    if (baud == 0) {
      error = "unsupported UART baud rate";
      return false;
    }
    const int descriptor = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK);
    if (descriptor < 0) {
      error = "open " + device_ + ": " + std::strerror(errno);
      return false;
    }
    descriptor_.reset(descriptor);
    termios options{};
    if (tcgetattr(descriptor_.get(), &options) != 0) {
      error = "tcgetattr: " + std::string(std::strerror(errno));
      return false;
    }
    cfmakeraw(&options);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    if (tcsetattr(descriptor_.get(), TCSANOW, &options) != 0) {
      error = "tcsetattr: " + std::string(std::strerror(errno));
      return false;
    }
    tcflush(descriptor_.get(), TCIOFLUSH);
    return true;
  }

  bool send(const std::vector<std::uint8_t> & frame, std::string & error) override
  {
    std::size_t offset = 0;
    while (offset < frame.size()) {
      pollfd event{descriptor_.get(), POLLOUT, 0};
      const int ready = ::poll(&event, 1, 100);
      if (ready <= 0) {
        error = ready == 0 ? "UART write timeout" : std::strerror(errno);
        return false;
      }
      const auto written = ::write(descriptor_.get(), frame.data() + offset, frame.size() - offset);
      if (written < 0) {
        if (errno == EAGAIN || errno == EINTR) {continue;}
        error = "UART write: " + std::string(std::strerror(errno));
        return false;
      }
      offset += static_cast<std::size_t>(written);
    }
    return true;
  }

  std::string name() const override {return "uart:" + device_;}

private:
  std::string device_;
  int baud_rate_;
  FileDescriptor descriptor_;
};

class SpiTransport final : public HardwareTransport
{
public:
  SpiTransport(std::string device, std::uint32_t speed_hz)
  : device_(std::move(device)), speed_hz_(speed_hz) {}

  bool open(std::string & error) override
  {
    const int descriptor = ::open(device_.c_str(), O_RDWR | O_CLOEXEC);
    if (descriptor < 0) {
      error = "open " + device_ + ": " + std::strerror(errno);
      return false;
    }
    descriptor_.reset(descriptor);
    std::uint8_t mode = SPI_MODE_0;
    std::uint8_t bits = 8;
    if (ioctl(descriptor_.get(), SPI_IOC_WR_MODE, &mode) < 0 ||
      ioctl(descriptor_.get(), SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
      ioctl(descriptor_.get(), SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0)
    {
      error = "configure SPI: " + std::string(std::strerror(errno));
      return false;
    }
    return true;
  }

  bool send(const std::vector<std::uint8_t> & frame, std::string & error) override
  {
    std::vector<std::uint8_t> received(frame.size());
    spi_ioc_transfer transfer{};
    transfer.tx_buf = reinterpret_cast<std::uintptr_t>(frame.data());
    transfer.rx_buf = reinterpret_cast<std::uintptr_t>(received.data());
    transfer.len = static_cast<std::uint32_t>(frame.size());
    transfer.speed_hz = speed_hz_;
    transfer.bits_per_word = 8;
    if (ioctl(descriptor_.get(), SPI_IOC_MESSAGE(1), &transfer) < 0) {
      error = "SPI transfer: " + std::string(std::strerror(errno));
      return false;
    }
    return true;
  }

  std::string name() const override {return "spi:" + device_;}

private:
  std::string device_;
  std::uint32_t speed_hz_;
  FileDescriptor descriptor_;
};

}  // namespace

std::unique_ptr<HardwareTransport> make_mock_transport()
{
  return std::make_unique<MockTransport>();
}

std::unique_ptr<HardwareTransport> make_uart_transport(
  const std::string & device, int baud_rate)
{
  return std::make_unique<UartTransport>(device, baud_rate);
}

std::unique_ptr<HardwareTransport> make_spi_transport(
  const std::string & device, std::uint32_t speed_hz)
{
  return std::make_unique<SpiTransport>(device, speed_hz);
}

}  // namespace embodied_agent_cpp
