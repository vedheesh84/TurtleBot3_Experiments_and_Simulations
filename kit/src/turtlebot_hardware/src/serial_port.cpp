// Copyright (c) 2024 ROS2 Learning Kit
// MIT License

/**
 * @file serial_port.cpp
 * @brief Serial port implementation for motor controller communication
 *
 * LEARNING NOTE:
 * This is a minimal serial port implementation.
 * Production code might use Boost.Asio or libserial for robustness.
 * We keep it simple here for learning purposes.
 */

#include "turtlebot_hardware/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>

namespace turtlebot_hardware
{

SerialPort::SerialPort()
: fd_(-1)
{
}

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & port, int baud_rate)
{
  // Open port
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  // Configure port
  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));

  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Set baud rate
  speed_t baud;
  switch (baud_rate) {
    case 9600: baud = B9600; break;
    case 19200: baud = B19200; break;
    case 38400: baud = B38400; break;
    case 57600: baud = B57600; break;
    case 115200: baud = B115200; break;
    default: baud = B115200; break;
  }
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  // 8N1 mode
  tty.c_cflag &= ~PARENB;  // No parity
  tty.c_cflag &= ~CSTOPB;  // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;       // 8 bits

  // No flow control
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  // Raw mode
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;

  // Read settings
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;  // 0.1 second timeout

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Clear buffers
  flush();

  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  read_buffer_.clear();
}

bool SerialPort::is_open() const
{
  return fd_ >= 0;
}

ssize_t SerialPort::write(const std::string & data)
{
  if (fd_ < 0) {
    return -1;
  }
  return ::write(fd_, data.c_str(), data.length());
}

std::optional<std::string> SerialPort::read_line(int timeout_ms)
{
  if (fd_ < 0) {
    return std::nullopt;
  }

  // Check for existing newline in buffer
  size_t pos = read_buffer_.find('\n');
  if (pos != std::string::npos) {
    std::string line = read_buffer_.substr(0, pos);
    read_buffer_.erase(0, pos + 1);
    return line;
  }

  // Wait for data with timeout
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(fd_, &read_fds);

  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int result = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
  if (result <= 0) {
    return std::nullopt;  // Timeout or error
  }

  // Read available data
  char buffer[256];
  ssize_t n = ::read(fd_, buffer, sizeof(buffer) - 1);
  if (n <= 0) {
    return std::nullopt;
  }

  buffer[n] = '\0';
  read_buffer_ += buffer;

  // Check for newline again
  pos = read_buffer_.find('\n');
  if (pos != std::string::npos) {
    std::string line = read_buffer_.substr(0, pos);
    read_buffer_.erase(0, pos + 1);
    return line;
  }

  return std::nullopt;
}

void SerialPort::flush()
{
  if (fd_ >= 0) {
    tcflush(fd_, TCIOFLUSH);
  }
  read_buffer_.clear();
}

}  // namespace turtlebot_hardware
