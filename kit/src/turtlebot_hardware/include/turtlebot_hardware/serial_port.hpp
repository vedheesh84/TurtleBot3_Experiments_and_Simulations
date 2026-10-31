// Copyright (c) 2024 ROS2 Learning Kit
// MIT License

#ifndef TURTLEBOT_HARDWARE__SERIAL_PORT_HPP_
#define TURTLEBOT_HARDWARE__SERIAL_PORT_HPP_

/**
 * @file serial_port.hpp
 * @brief Simple serial port wrapper for hardware communication
 *
 * LEARNING OBJECTIVES:
 * - Understand serial communication basics
 * - See how ROS2 hardware drivers work at low level
 * - Learn about file descriptors and termios
 *
 * PROTOCOL:
 * Commands sent to motor controller:
 *   VEL,<left_vel>,<right_vel>\n
 *
 * Responses from motor controller:
 *   ENC,<left_count>,<right_count>\n
 *
 * This matches the gripper_car_ws protocol for compatibility.
 */

#include <string>
#include <optional>

namespace turtlebot_hardware
{

class SerialPort
{
public:
  SerialPort();
  ~SerialPort();

  /**
   * @brief Open serial port
   * @param port Device path (e.g., /dev/ttyACM0)
   * @param baud_rate Baud rate (e.g., 115200)
   * @return true if successful
   */
  bool open(const std::string & port, int baud_rate);

  /**
   * @brief Close serial port
   */
  void close();

  /**
   * @brief Check if port is open
   */
  bool is_open() const;

  /**
   * @brief Write string to serial port
   * @param data String to write
   * @return Number of bytes written, or -1 on error
   */
  ssize_t write(const std::string & data);

  /**
   * @brief Read line from serial port (blocking with timeout)
   * @param timeout_ms Timeout in milliseconds
   * @return Line read, or empty optional on timeout/error
   */
  std::optional<std::string> read_line(int timeout_ms = 100);

  /**
   * @brief Flush input and output buffers
   */
  void flush();

private:
  int fd_;  // File descriptor
  std::string read_buffer_;
};

}  // namespace turtlebot_hardware

#endif  // TURTLEBOT_HARDWARE__SERIAL_PORT_HPP_
