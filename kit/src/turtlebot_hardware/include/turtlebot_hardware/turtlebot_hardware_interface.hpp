// Copyright (c) 2024 ROS2 Learning Kit
// MIT License

#ifndef TURTLEBOT_HARDWARE__TURTLEBOT_HARDWARE_INTERFACE_HPP_
#define TURTLEBOT_HARDWARE__TURTLEBOT_HARDWARE_INTERFACE_HPP_

/**
 * @file turtlebot_hardware_interface.hpp
 * @brief ros2_control hardware interface for TurtleBot AMR
 *
 * LEARNING OBJECTIVES (builds on learning_lifecycle):
 * - Understand hardware_interface::SystemInterface base class
 * - See lifecycle callback pattern in C++
 * - Learn command/state interface exports
 * - Practice minimal C++ for ROS2 hardware
 *
 * ARCHITECTURE:
 *
 *   Controller Manager
 *         |
 *         v
 *   diff_drive_controller
 *         |
 *         v
 *   Command Interfaces    State Interfaces
 *   (velocity)            (position, velocity)
 *         |                      ^
 *         v                      |
 *   +----------------------------------+
 *   |  TurtlebotHardwareInterface      |
 *   |  --------------------------------|
 *   |  read():  Serial -> State        |
 *   |  write(): Command -> Serial      |
 *   +----------------------------------+
 *         |
 *         v
 *   Serial Port (/dev/ttyACM0)
 *         |
 *         v
 *   Motor Controller (Arduino)
 *
 * LIFECYCLE:
 *   on_init()        -> Parse URDF parameters
 *   on_configure()   -> Open serial port
 *   on_activate()    -> Start communication
 *   on_deactivate()  -> Stop communication
 *   on_cleanup()     -> Close serial port
 */

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "turtlebot_hardware/serial_port.hpp"

namespace turtlebot_hardware
{

class TurtlebotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TurtlebotHardwareInterface)

  /**
   * @brief Initialize from URDF hardware info
   *
   * Called once when the hardware interface is loaded.
   * Parse parameters from URDF <param> tags.
   *
   * @param info Hardware configuration from URDF
   * @return CallbackReturn::SUCCESS or ERROR
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Configure the hardware (open connections)
   *
   * Called when transitioning to INACTIVE state.
   * Open serial port but don't start communication.
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activate the hardware (start communication)
   *
   * Called when transitioning to ACTIVE state.
   * Start reading encoders and accepting commands.
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate the hardware (stop communication)
   *
   * Called when transitioning from ACTIVE state.
   * Stop motors and cease communication.
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Cleanup the hardware (close connections)
   *
   * Called when transitioning to UNCONFIGURED state.
   * Close serial port.
   */
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Export state interfaces for controllers to read
   *
   * Controllers read joint states through these interfaces.
   * We export position and velocity for each wheel.
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Export command interfaces for controllers to write
   *
   * Controllers write commands through these interfaces.
   * We export velocity command for each wheel.
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Read state from hardware
   *
   * Called every control cycle by controller_manager.
   * Read encoder counts and update position/velocity.
   *
   * @param time Current time
   * @param period Time since last read
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Write commands to hardware
   *
   * Called every control cycle by controller_manager.
   * Send velocity commands to motors.
   *
   * @param time Current time
   * @param period Time since last write
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial communication
  std::unique_ptr<SerialPort> serial_;
  std::string serial_port_;
  int baud_rate_;

  // Robot parameters
  double wheel_radius_;
  double wheel_separation_;
  int encoder_cpr_;  // Counts per revolution

  // Joint states (read from hardware)
  double left_wheel_pos_;
  double left_wheel_vel_;
  double right_wheel_pos_;
  double right_wheel_vel_;

  // Joint commands (written to hardware)
  double left_wheel_cmd_;
  double right_wheel_cmd_;

  // Encoder tracking
  int64_t left_encoder_count_;
  int64_t right_encoder_count_;
  int64_t prev_left_encoder_count_;
  int64_t prev_right_encoder_count_;

  // State tracking
  bool is_active_;

  // Helper methods
  void parse_encoder_message(const std::string & msg);
  void send_velocity_command();
};

}  // namespace turtlebot_hardware

#endif  // TURTLEBOT_HARDWARE__TURTLEBOT_HARDWARE_INTERFACE_HPP_
