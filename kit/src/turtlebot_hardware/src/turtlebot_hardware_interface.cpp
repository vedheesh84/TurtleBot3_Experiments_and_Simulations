// Copyright (c) 2024 ROS2 Learning Kit
// MIT License

/**
 * @file turtlebot_hardware_interface.cpp
 * @brief ros2_control hardware interface implementation
 *
 * LEARNING OBJECTIVES:
 * - See complete ros2_control hardware interface implementation
 * - Understand lifecycle callback sequence
 * - Learn state/command interface patterns
 * - Practice encoder-to-position conversion
 *
 * LIFECYCLE SEQUENCE:
 *   1. on_init()       - Parse parameters from URDF
 *   2. on_configure()  - Open serial port
 *   3. on_activate()   - Start control loop
 *   4. [read/write in loop]
 *   5. on_deactivate() - Stop motors
 *   6. on_cleanup()    - Close port
 *
 * PROTOCOL (matches gripper_car_ws):
 *   Send: VEL,<left_rad_s>,<right_rad_s>\n
 *   Recv: ENC,<left_count>,<right_count>\n
 */

#include "turtlebot_hardware/turtlebot_hardware_interface.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace turtlebot_hardware
{

hardware_interface::CallbackReturn TurtlebotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Call base class implementation first
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse parameters from URDF <param> tags
  // These are defined in ros2_control.urdf.xacro
  serial_port_ = info_.hardware_parameters["serial_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);
  encoder_cpr_ = std::stoi(info_.hardware_parameters["encoder_cpr"]);

  RCLCPP_INFO(
    rclcpp::get_logger("TurtlebotHardwareInterface"),
    "Initialized with: port=%s, baud=%d, wheel_radius=%.3f, wheel_sep=%.3f, encoder_cpr=%d",
    serial_port_.c_str(), baud_rate_, wheel_radius_, wheel_separation_, encoder_cpr_);

  // Initialize state variables
  left_wheel_pos_ = 0.0;
  left_wheel_vel_ = 0.0;
  right_wheel_pos_ = 0.0;
  right_wheel_vel_ = 0.0;

  left_wheel_cmd_ = 0.0;
  right_wheel_cmd_ = 0.0;

  left_encoder_count_ = 0;
  right_encoder_count_ = 0;
  prev_left_encoder_count_ = 0;
  prev_right_encoder_count_ = 0;

  is_active_ = false;

  // Verify joint configuration
  // We expect exactly 2 joints: left_wheel_joint, right_wheel_joint
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("TurtlebotHardwareInterface"),
      "Expected 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Verify each joint has velocity command and position/velocity state interfaces
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("TurtlebotHardwareInterface"),
        "Joint '%s' must have exactly one velocity command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(
        rclcpp::get_logger("TurtlebotHardwareInterface"),
        "Joint '%s' must have position and velocity state interfaces",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TurtlebotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("TurtlebotHardwareInterface"),
    "Configuring hardware interface...");

  // Create and open serial port
  serial_ = std::make_unique<SerialPort>();

  if (!serial_->open(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("TurtlebotHardwareInterface"),
      "Failed to open serial port %s", serial_port_.c_str());

    // In simulation or testing without hardware, we can still proceed
    // This allows testing the interface without physical hardware
    RCLCPP_WARN(
      rclcpp::get_logger("TurtlebotHardwareInterface"),
      "Continuing without serial connection (mock mode)");
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("TurtlebotHardwareInterface"),
      "Serial port %s opened successfully", serial_port_.c_str());
    serial_->flush();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TurtlebotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("TurtlebotHardwareInterface"),
    "Activating hardware interface...");

  // Reset encoder counts
  prev_left_encoder_count_ = left_encoder_count_;
  prev_right_encoder_count_ = right_encoder_count_;

  // Stop motors initially
  left_wheel_cmd_ = 0.0;
  right_wheel_cmd_ = 0.0;
  send_velocity_command();

  is_active_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger("TurtlebotHardwareInterface"),
    "Hardware interface activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TurtlebotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("TurtlebotHardwareInterface"),
    "Deactivating hardware interface...");

  is_active_ = false;

  // Stop motors
  left_wheel_cmd_ = 0.0;
  right_wheel_cmd_ = 0.0;
  send_velocity_command();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TurtlebotHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("TurtlebotHardwareInterface"),
    "Cleaning up hardware interface...");

  if (serial_) {
    serial_->close();
    serial_.reset();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
TurtlebotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export position and velocity for left wheel
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &left_wheel_pos_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_wheel_vel_));

  // Export position and velocity for right wheel
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[1].name, hardware_interface::HW_IF_POSITION, &right_wheel_pos_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_wheel_vel_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TurtlebotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Export velocity command for left wheel
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_wheel_cmd_));

  // Export velocity command for right wheel
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_wheel_cmd_));

  return command_interfaces;
}

hardware_interface::return_type TurtlebotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!is_active_) {
    return hardware_interface::return_type::OK;
  }

  // Read encoder data from serial port
  if (serial_ && serial_->is_open()) {
    auto line = serial_->read_line(10);  // 10ms timeout
    if (line.has_value()) {
      parse_encoder_message(line.value());
    }
  }

  // Calculate velocity from encoder changes
  double dt = period.seconds();
  if (dt > 0.0) {
    // Encoder counts to radians
    double counts_to_rad = 2.0 * M_PI / static_cast<double>(encoder_cpr_);

    int64_t left_delta = left_encoder_count_ - prev_left_encoder_count_;
    int64_t right_delta = right_encoder_count_ - prev_right_encoder_count_;

    left_wheel_vel_ = (left_delta * counts_to_rad) / dt;
    right_wheel_vel_ = (right_delta * counts_to_rad) / dt;

    left_wheel_pos_ += left_delta * counts_to_rad;
    right_wheel_pos_ += right_delta * counts_to_rad;

    prev_left_encoder_count_ = left_encoder_count_;
    prev_right_encoder_count_ = right_encoder_count_;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurtlebotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!is_active_) {
    return hardware_interface::return_type::OK;
  }

  send_velocity_command();

  return hardware_interface::return_type::OK;
}

void TurtlebotHardwareInterface::parse_encoder_message(const std::string & msg)
{
  // Expected format: ENC,<left_count>,<right_count>
  if (msg.substr(0, 4) != "ENC,") {
    return;
  }

  std::istringstream iss(msg.substr(4));
  std::string token;

  if (std::getline(iss, token, ',')) {
    left_encoder_count_ = std::stoll(token);
  }
  if (std::getline(iss, token, ',')) {
    right_encoder_count_ = std::stoll(token);
  }
}

void TurtlebotHardwareInterface::send_velocity_command()
{
  if (!serial_ || !serial_->is_open()) {
    return;
  }

  // Format: VEL,<left_rad_s>,<right_rad_s>\n
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "VEL," << left_wheel_cmd_ << "," << right_wheel_cmd_ << "\n";

  serial_->write(oss.str());
}

}  // namespace turtlebot_hardware

// Register plugin with pluginlib
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  turtlebot_hardware::TurtlebotHardwareInterface,
  hardware_interface::SystemInterface)
