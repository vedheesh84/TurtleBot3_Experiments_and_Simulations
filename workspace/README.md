# turtlebot3_ws

**Level 1 Capstone Robot Workspace & Autonomous Mobile Robotics Platform**

An industry-standard differential-drive mobile robotics platform integrating hardware abstraction, physics simulation, action servers, multi-robot namespacing, SLAM mapping, and Nav2 autonomous navigation.

---

## 1. Overview & Identity

`turtlebot3_ws` serves as the **Level 1 Capstone Proving Ground** for the ROS2 curriculum. It provides a complete, working cyber-physical embodiment of all concepts taught across Articles 1–25 (Nodes, Topics, Services, Actions, Parameters, Launch Design, Namespacing, Lifecycle, TF Trees, Simulation, Debugging, and Systems Thinking).

Furthermore, it acts as the foundational bridge and gateway to specialized Autonomous Mobile Robotics (AMR), Cartographer 2D SLAM, and Navigation2 (Nav2) stacks.

---

## 2. Workspace Package Architecture

```text
turtlebot3_ws/
├── SYSTEM_ARCHITECTURE.md                # In-depth architectural & coupling analysis
├── CAPSTONE_DEMONSTRATION_GUIDE.md       # Step-by-step 6-stage capstone walkthrough
├── README.md                             # Primary workspace documentation
└── src/
    ├── DynamixelSDK/                     # Motor driver SDK for XL430 actuators
    ├── turtlebot3/
    │   ├── turtlebot3_bringup/           # Hardware bringup, state publisher, RViz
    │   ├── turtlebot3_cartographer/      # 2D LiDAR Cartographer SLAM configuration
    │   ├── turtlebot3_description/       # URDF/Xacro models for Burger, Waffle, Waffle Pi
    │   ├── turtlebot3_example/           # Action server/client (patrol), obstacle safety filter
    │   ├── turtlebot3_navigation2/       # Nav2 stack parameters, costmaps, and maps
    │   ├── turtlebot3_node/              # C++ OpenCR hardware bridge & diff_drive_controller
    │   └── turtlebot3_teleop/            # Keyboard velocity control
    ├── turtlebot3_msgs/                  # Message, service, and action schemas (Patrol.action)
    └── turtlebot3_simulations/
        ├── turtlebot3_fake_node/         # Lightweight kinematic simulator (no physics)
        ├── turtlebot3_gazebo/            # Gazebo worlds (world, house, multi-robot, autorace)
        └── turtlebot3_manipulation_gazebo/# OpenManipulator-X arm + mobile base hybrid URDF
```

---

## 3. System Architecture & Communication

Detailed system design, data flows, and coupling hotspot analyses are documented in [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md).

```text
  [teleop / Nav2 / patrol] ──▶ /cmd_vel (Twist)
                                  │
                                  ▼
                         [turtlebot3_node] ──▶ OpenCR / Actuators
                                  │
                                  ├─▶ /joint_states ──▶ [diff_drive_controller]
                                  ├─▶ /imu                               │
                                  ├─▶ /sensor_state                      ▼
                                  └─▶ /battery_state            /odom, TF (odom ➔ base_footprint)
                                                                         ▲
  [LIDAR Driver / Gazebo] ────▶ /scan (LaserScan) ───────────────────────┤
                                                                         │
                                                              [Cartographer SLAM]
                                                                         │
                                                                         ▼
                                                                /map, TF (map ➔ odom)
```

---

## 4. Quick Start

```bash
# 1. Navigate to workspace and build
cd "02 — Domains/ROS2/AMR_ws/turtlebot3_ws"
colcon build --symlink-install
source install/setup.bash

# 2. Set the default robot model
export TURTLEBOT3_MODEL=burger

# 3. Launch Gazebo Simulation World
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 4. In a second terminal: Launch Keyboard Teleop
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## 5. Capstone Demonstration Progression

To explore all 25 curriculum concepts running inside this embodied mobile robot, follow the comprehensive [CAPSTONE_DEMONSTRATION_GUIDE.md](CAPSTONE_DEMONSTRATION_GUIDE.md):

1. **Stage 1 (Kinematic Testing)**: Lightweight testing via `turtlebot3_fake_node` (no physics engine required).
2. **Stage 2 (Physics Simulation)**: Full Gazebo world bringup with laser scanners, contact physics, and camera feeds.
3. **Stage 3 (ROS2 Actions in Action)**: Run `/patrol` Action Server and send square-patrol goals with real-time feedback.
4. **Stage 4 (Multi-Robot Namespaces & Scaling)**: Spawn isolated multi-robot instances (`tb3_0`, `tb3_1`) using `multi_robot.launch.py`.
5. **Stage 5 (Safety & Introspection)**: Intercept `/cmd_vel` with `turtlebot3_obstacle_detection` to prevent collisions.
6. **Stage 6 (AMR Gateway — SLAM & Nav2)**: 2D Cartographer mapping and autonomous waypoint navigation.

---

## 6. License & Origin

Developed by ROBOTIS and adapted for the Intelligent Systems Knowledge Ecosystem under the Apache 2.0 License.
