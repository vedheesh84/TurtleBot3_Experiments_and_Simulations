# ros2_turtlebot_kit - System Architecture

## Package Overview

| Package | Layer | Purpose |
|---------|-------|---------|
| `turtlebot_description` | System | Robot URDF/Xacro, TF tree, hardware interfaces |
| `turtlebot_hardware` | Hardware | C++ ros2_control plugin for motor controller |
| `turtlebot_bringup` | System | Launch orchestration, mode switching |
| `turtlebot_localization` | Perception | EKF sensor fusion (wheel odom + IMU) |
| `turtlebot_slam` | Perception | SLAM Toolbox for mapping/localization |
| `turtlebot_navigation` | Planning/Control | Nav2 stack (planner, controller, behaviors) |
| `turtlebot_demos` | Application | Progressive demos + failure injection |

---

## 1. Package-by-Package Analysis

### turtlebot_description (System Layer)

**Purpose:** Robot structure, kinematics, and hardware abstraction

**TF Tree:**
```
map (from SLAM/localization)
└── odom (from EKF or SLAM)
    └── base_link
        ├── base_footprint (virtual, for Nav2)
        ├── left_wheel_link (continuous joint)
        ├── right_wheel_link (continuous joint)
        ├── front_caster_link
        ├── rear_caster_link
        ├── laser_frame (LiDAR mount, Z=0.15m)
        ├── imu_link (chassis center)
        └── camera_link → camera_optical_link
```

**Physical Properties:**
- Chassis: 0.30m × 0.25m × 0.10m, 5.0 kg
- Wheels: 0.05m radius, 0.20m separation, 0.5 kg each
- Casters: 0.02m radius (passive), 0.1 kg each

**Hardware Interfaces (ros2_control):**
- Command: velocity (rad/s) to wheels
- State: position, velocity from encoders

**Controllers:**
- `joint_state_broadcaster` @ 50Hz
- `diff_drive_controller` for base motion

---

### turtlebot_hardware (Hardware Layer)

**Purpose:** Real robot motor controller interface via ros2_control

**Serial Protocol:**
```
TX: VEL,<left_rad_s>,<right_rad_s>\n
RX: ENC,<left_count>,<right_count>\n
```

**Parameters:**
| Parameter | Default | Purpose |
|-----------|---------|---------|
| `serial_port` | /dev/ttyACM0 | USB connection |
| `baud_rate` | 115200 | Communication speed |
| `wheel_radius` | 0.05 | Meters |
| `wheel_separation` | 0.20 | Meters |
| `encoder_cpr` | 360 | Counts per revolution |

**Lifecycle:**
- `on_configure()`: Open serial port
- `on_activate()`: Reset encoders, start control
- `read()`: Parse encoder messages, update states
- `write()`: Send velocity commands

---

### turtlebot_localization (Perception Layer)

**Purpose:** Sensor fusion combining wheel odometry + IMU

**Node:** `ekf_filter_node` (robot_localization)

**Input Topics:**
| Topic | Message Type | Used For |
|-------|--------------|----------|
| `/diff_drive_controller/odom` | Odometry | Linear/angular velocity |
| `/imu/data` | Imu | Orientation, angular velocity |

**Output Topics:**
| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/odometry/filtered` | Odometry | Fused state estimate |
| `/tf` (odom→base_link) | Transform | Localization |

**Sensor Fusion Strategy:**
```yaml
Wheel Odometry:
  - Use: linear vx, angular vyaw
  - Ignore: position (integrate velocity instead)

IMU:
  - Use: roll, pitch, yaw orientation
  - Use: angular velocity (all axes)
  - Ignore: linear acceleration (too noisy)
```

---

### turtlebot_slam (Perception Layer)

**Purpose:** SLAM Toolbox for mapping and localization

**Modes:**
- **Mapping:** `async_slam_toolbox_node` - builds occupancy grid
- **Localization:** `localization_slam_toolbox_node` - uses saved map

**Key Parameters:**
```yaml
minimum_travel_distance: 0.3 m
minimum_travel_heading: 0.3 rad
do_loop_closing: true
resolution: 0.05 m (5 cm cells)
transform_publish_period: 0.02 s (50 Hz)
```

**Output:**
- `/map` - Occupancy grid
- `/tf` (map→odom) - Global localization

---

### turtlebot_navigation (Planning/Control Layer)

**Purpose:** Nav2 stack for autonomous navigation

**Nodes:**
| Node | Purpose |
|------|---------|
| `bt_navigator` | Behavior tree orchestration |
| `map_server` | Static map provider |
| `planner_server` | Global path planning (A*) |
| `controller_server` | Local trajectory (DWB) |
| `behavior_server` | Recovery actions |
| `lifecycle_manager` | State coordination |

**Costmap Layers:**
1. Static Layer (from /map)
2. Obstacle Layer (from /scan)
3. Inflation Layer (robot radius expansion)

**Velocity Limits:**
```yaml
max_vel_x: 0.26 m/s
max_vel_theta: 1.0 rad/s
acc_lim_x: 2.5 m/s²
acc_lim_theta: 3.2 rad/s²
```

**Goal Tolerance:**
- xy_goal_tolerance: 0.25 m
- yaw_goal_tolerance: 0.25 rad

---

### turtlebot_demos (Application Layer)

**Demo Progression:**
1. `demo_01_teleop_tf` - Teleop + TF visualization
2. `demo_02_ros2_control` - Controller spawning
3. `demo_03_sensor_fusion` - EKF demonstration
4. `demo_04_slam_mapping` - SLAM map creation
5. `demo_05_nav2_basics` - Navigation goals
6. `demo_06_full_autonomy` - Complete autonomous navigation

**Failure Injection:**
- `break_tf.py` - TF chain errors (wrong parent, stale, duplicate)
- `break_odom.py` - Odometry corruption (drift, noise, wrong frame)
- `break_costmap.py` - Costmap issues (phantom obstacles, clearing)

---

## 2. System Architecture Analysis

### Data Flow Diagram

```
HARDWARE/SIMULATION
├── Wheels + Encoders → serial
├── LiDAR → /scan
├── IMU → /imu/data
└── Camera → /camera/image_raw
        ↓
ROBOT HARDWARE INTERFACE (ros2_control)
└── /diff_drive_controller/odom
        ↓
SENSOR FUSION (EKF)
├── Input: /diff_drive_controller/odom, /imu/data
└── Output: /odometry/filtered, /tf (odom→base_link)
        ↓
LOCALIZATION (SLAM Toolbox)
├── Input: /scan, /odometry/filtered
└── Output: /map, /tf (map→odom)
        ↓
PLANNING (Nav2 Planner)
├── Input: /map, goal pose
└── Output: global path
        ↓
CONTROL (Nav2 Controller - DWB)
├── Input: path, /scan, /odometry/filtered
└── Output: /cmd_vel
        ↓
MOTOR COMMANDS
└── diff_drive_controller → hardware
```

### TF Publishing Sources

| Transform | Publisher | Rate |
|-----------|-----------|------|
| base_link → sensors | robot_state_publisher | 10 Hz |
| odom → base_link | EKF (robot_localization) | 50 Hz |
| map → odom | SLAM Toolbox | 50 Hz |

---

## 3. Coordination Patterns

### Lifecycle Management
```
Controller Manager
├── unconfigured → inactive → active
└── Manages: joint_state_broadcaster, diff_drive_controller

Nav2 Stack
├── lifecycle_manager coordinates all Nav2 nodes
└── Single transition activates entire stack
```

### Launch Sequencing (TimerActions)
```
t=0s:  robot_state_publisher (TF tree)
t=2s:  Gazebo simulator
t=5s:  spawn_entity (robot in Gazebo)
t=7s:  controller spawners
t=8s:  EKF localization
t=10s: SLAM/Localization
t=12s: Nav2 stack
t=14s: teleop (optional)
t=16s: RViz
```

### Hardware Abstraction
```
sim_mode=true  → GazeboSystem plugin
sim_mode=false → TurtlebotHardwareInterface plugin
```

---

## 4. Agent-Based Analysis

### What Each Component Knows

| Component | Local Knowledge | Global Knowledge |
|-----------|-----------------|------------------|
| Hardware Interface | Serial protocol, encoders | None |
| EKF | Sensor fusion math, covariance | None |
| SLAM Toolbox | Scan matching, pose graph | Map structure |
| Nav2 Planner | Graph search algorithms | Costmap |
| Nav2 Controller | DWB velocity sampling | Local obstacles |

### What Each Component Assumes

| Component | Assumptions |
|-----------|-------------|
| Hardware Interface | Serial protocol format, encoder CPR |
| EKF | IMU and odom have accurate covariance |
| SLAM | LiDAR provides accurate scans |
| Nav2 | Complete TF chain available |

### Coordination Emergence

1. **TF Chain:** All nodes depend on consistent transform tree
2. **Topic Contracts:** Message types and frame_ids must match
3. **Lifecycle:** Sequential startup ensures dependencies ready
4. **No Explicit Handshakes:** Pub/sub with last-message semantics

---

## 5. Coupling Analysis

### Tight Coupling

| Coupling | Risk |
|----------|------|
| EKF ↔ diff_drive_controller | `enable_odom_tf: false` required to avoid conflict |
| Nav2 ↔ TF chain | Missing transform causes navigation failure |
| SLAM ↔ /scan | Wrong frame_id breaks localization |

### Loose Coupling

| Component | Why Loose |
|-----------|-----------|
| Costmap layers | Plugin architecture |
| Planner/Controller | YAML configuration |
| SLAM modes | Launch parameter selection |

---

## 6. Reusability Analysis

### Highly Reusable (>80%)

| Component | Reason |
|-----------|--------|
| turtlebot_navigation | Standard Nav2 patterns |
| turtlebot_localization | Generic EKF configuration |
| turtlebot_slam | SLAM Toolbox is robot-agnostic |
| turtlebot_demos | Learning patterns applicable |

### Robot-Specific (<40%)

| Component | Reason |
|-----------|--------|
| turtlebot_hardware | Serial protocol specific |
| URDF dimensions | Physical robot measurements |
| Controller gains | Tuned for this robot |

### Reusability Score

```
turtlebot_slam:       ★★★★★ (100%) - Any 2D LiDAR robot
turtlebot_localization: ★★★★★ (95%)  - Any wheeled robot
turtlebot_navigation: ★★★★☆ (85%)  - Any mobile robot
turtlebot_bringup:    ★★★★☆ (75%)  - Similar architecture
turtlebot_description: ★★★☆☆ (60%)  - Similar chassis
turtlebot_hardware:   ★★☆☆☆ (25%)  - Template for pattern
turtlebot_demos:      ★★★★★ (95%)  - Learning patterns
```

---

## 7. Topic Summary

| Topic | Publisher | Subscriber(s) | Type |
|-------|-----------|---------------|------|
| `/scan` | LiDAR driver | SLAM, Nav2 | LaserScan |
| `/imu/data` | IMU driver | EKF | Imu |
| `/diff_drive_controller/odom` | diff_drive | EKF | Odometry |
| `/odometry/filtered` | EKF | Nav2, SLAM | Odometry |
| `/map` | SLAM/map_server | Nav2 | OccupancyGrid |
| `/cmd_vel` | Nav2/teleop | diff_drive | Twist |
| `/joint_states` | joint_state_broadcaster | robot_state_publisher | JointState |

---

## 8. Parameter Summary

| Node | Key Parameters |
|------|----------------|
| Hardware Interface | `serial_port`, `baud_rate`, `wheel_radius`, `wheel_separation`, `encoder_cpr` |
| EKF | Process noise Q, measurement noise R, sensor enable booleans |
| SLAM | `resolution`, `minimum_travel_distance`, `do_loop_closing` |
| Nav2 Controller | `max_vel_x`, `max_vel_theta`, `acc_lim_*`, goal tolerances |
| diff_drive_controller | `enable_odom_tf: false`, velocity limits |

---

## 9. Critical Design Decisions

### Why `enable_odom_tf: false`?
- EKF publishes odom→base_link with sensor fusion
- diff_drive_controller would conflict if also publishing
- Only one node can own a TF transform

### Why SLAM publishes map→odom (not map→base_link)?
- TF chain: map → odom → base_link
- SLAM corrects odometry drift via loop closure
- EKF handles odom→base_link (local motion)

### Why Sensor Fusion Instead of Single Sensor?
- Wheel odometry: Good position, poor heading
- IMU: Good heading, no position
- Combined: Best of both sensors

---

## 10. Integration Complexity Hotspots

| Hotspot | Issue | Mitigation |
|---------|-------|------------|
| TF chain | Any break causes system failure | view_frames debugging |
| Startup timing | Race conditions | TimerActions with delays |
| Covariance tuning | Poor fusion if wrong | odom_drift_visualizer demo |
| Frame IDs | Wrong names cause lookup failures | Consistent naming convention |
