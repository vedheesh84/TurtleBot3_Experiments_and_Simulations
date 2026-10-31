# TurtleBot AMR Learning Kit

A comprehensive ROS2 learning workspace for Autonomous Mobile Robots (AMR), focusing on SLAM, Navigation, and ros2_control.

## Prerequisites

**Builds on ROS2_kits_ws foundation.** Complete these learning packages first:

| Foundation Package | Concepts | Required For |
|-------------------|----------|--------------|
| learning_tf | TF tree, frames, transforms | Demo 01 |
| learning_lifecycle | Node lifecycle, state machines | Demo 02, 04 |
| learning_comms | Topics, services, actions | Demo 03 |
| learning_execution | Behavior trees, executors | Demo 05 |
| learning_integration | System composition | Demo 06 |

## Workspace Structure

```
ros2_turtlebot_kit/
├── src/
│   ├── turtlebot_description/    # URDF, ros2_control, Gazebo plugins
│   ├── turtlebot_hardware/       # C++ ros2_control hardware interface
│   ├── turtlebot_bringup/        # Launch coordination
│   ├── turtlebot_localization/   # EKF sensor fusion
│   ├── turtlebot_slam/           # SLAM Toolbox wrapper
│   ├── turtlebot_navigation/     # Nav2 configuration
│   └── turtlebot_demos/          # Progressive learning demos
└── README.md
```

## Quick Start

```bash
# Build
cd ros2_turtlebot_kit
colcon build

# Source
source install/setup.bash

# Start Demo 01 (Teleop + TF)
ros2 launch turtlebot_demos demo_01_teleop_tf.launch.py
```

## Progressive Demo Sequence

| Demo | Title | Focus | Commands |
|------|-------|-------|----------|
| 01 | Teleop + TF | TF tree visualization | `ros2 run tf2_tools view_frames` |
| 02 | ros2_control | Controller lifecycle | `ros2 control list_controllers` |
| 03 | Sensor Fusion | EKF, odom vs filtered | `ros2 run turtlebot_demos odom_drift_visualizer` |
| 04 | SLAM Mapping | Pose graph, loop closure | `ros2 run turtlebot_slam save_map.py` |
| 05 | Nav2 Basics | Costmaps, planning | `ros2 run turtlebot_demos goal_sender` |
| 06 | Full Autonomy | System integration | Failure injection scripts |

## TF Tree

```
map
 └─ odom                    (published by SLAM/AMCL)
     └─ base_link           (published by EKF)
         ├─ laser_frame
         ├─ imu_link
         ├─ left_wheel_link
         └─ right_wheel_link
```

## Key Configuration Files

| File | Purpose |
|------|---------|
| `turtlebot_hardware/config/ros2_controllers.yaml` | Controller definitions |
| `turtlebot_localization/config/ekf_params.yaml` | EKF sensor fusion |
| `turtlebot_slam/config/mapping_params.yaml` | SLAM Toolbox mapping |
| `turtlebot_navigation/config/nav2_params.yaml` | Nav2 full stack |

## Failure Injection (Learning)

Break things intentionally to learn debugging:

```bash
# Break TF tree
ros2 run turtlebot_demos break_tf --mode duplicate

# Break odometry
ros2 run turtlebot_demos break_odom --mode drift

# Break costmap
ros2 run turtlebot_demos break_costmap --mode phantom_obstacles
```

## Hardware Interface & Arduino Firmware

The `turtlebot_hardware` package implements a minimal C++ ros2_control plugin:

- **Protocol**: Serial communication at 115200 baud with `VEL,<left>,<right>\n` commands
- **Feedback**: Encoder data stream at 50 Hz via `ENC,<left>,<right>\n`
- **Lifecycle**: Full lifecycle support (configure, activate, deactivate)

### Arduino Firmware
Flashing the physical microcontroller (Arduino Uno / Mega / ESP32 / Teensy):
- Source file: [`arduino/turtlebot_motor_controller/turtlebot_motor_controller.ino`](arduino/turtlebot_motor_controller/turtlebot_motor_controller.ino)
- Supports dual DC motor drivers (PWM/DIR), quadrature encoder interrupts, and a 50 Hz PID control loop.
- Built-in test emulation mode: send `TEST,ON\n` over serial (or enable `#define SIMULATION_TEST_MODE`) to test hardware communication without physical motors connected.

### Pseudo-Hardware Serial Emulator (Hardware-in-the-Loop Mock)
For automated testing or desktop development without physical microcontrollers:
```bash
# Start pseudo-hardware emulator (creates virtual serial port /tmp/tty_turtlebot_fake)
python3 scripts/pseudo_arduino_emulator.py --port /tmp/tty_turtlebot_fake

# In another terminal, launch hardware bringup connected to virtual port
ros2 launch turtlebot_bringup hardware.launch.py serial_port:=/tmp/tty_turtlebot_fake
```

## Simulation vs Hardware

```bash
# Simulation (Gazebo)
ros2 launch turtlebot_bringup simulation.launch.py

# Hardware (real robot or pseudo-hardware emulator)
ros2 launch turtlebot_bringup hardware.launch.py serial_port:=/dev/ttyACM0
```

The URDF uses `sim_mode` argument to switch between simulation plugins and real hardware interface.

## Dependencies

- ROS2 Humble or later
- Nav2
- SLAM Toolbox
- robot_localization
- ros2_control
- Gazebo (for simulation)

## License

MIT
