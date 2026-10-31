# turtlebot3_ws - System Architecture

## Package Overview

| Package | Layer | Purpose |
|---------|-------|---------|
| `turtlebot3_msgs` | Interface | Custom messages, services, actions |
| `turtlebot3_node` | Hardware | OpenCR communication, sensors, control |
| `turtlebot3_bringup` | System | Launch orchestration |
| `turtlebot3_description` | Description | URDF/Xacro robot models |
| `turtlebot3_teleop` | HCI | Keyboard teleoperation |
| `turtlebot3_example` | Application | Demo applications |
| `turtlebot3_navigation2` | Planning | Nav2 integration |
| `turtlebot3_cartographer` | Perception | SLAM configuration |
| `turtlebot3_gazebo` | Simulation | Gazebo physics simulation |
| `turtlebot3_fake_node` | Testing | Kinematic simulation (no physics) |

---

## 1. Package-by-Package Analysis

### turtlebot3_msgs (Interface Layer)

**Purpose:** Define custom data contracts

**Messages:**
```yaml
SensorState.msg:
  - Bumpers, cliff, sonar, illumination
  - Motor encoders (left/right)
  - Battery voltage, LEDs, buttons

Sound.msg:
  - OFF=0, ON=1, LOW_BATTERY=2, ERROR=3, BUTTON1=4, BUTTON2=5

VersionInfo.msg:
  - Hardware/firmware/software versions
```

**Services:**
```yaml
Sound.srv: uint8 value → success, message
Goal.srv: pose_x, pose_y → success
Dqn.srv: action, init → state[], reward, done
```

**Actions:**
```yaml
Patrol.action:
  Goal: geometry_msgs/Vector3
  Result: string
  Feedback: string state
```

---

### turtlebot3_node (Hardware Layer)

**Purpose:** Hardware abstraction and low-level control

**Architecture:** Monolithic C++ node with composite pattern

**Primary Node: TurtleBot3**

**Topics Published (20 Hz):**
| Topic | Type | Purpose |
|-------|------|---------|
| `/battery_state` | BatteryState | Voltage, percentage |
| `/imu` | Imu | 9-axis (gyro, accel, mag) |
| `/magnetic_field` | MagneticField | Magnetometer |
| `/joint_states` | JointState | Wheel positions/velocities |
| `/sensor_state` | SensorState | Bumpers, cliff, sonar |

**Topics Subscribed:**
| Topic | Type | Purpose |
|-------|------|---------|
| `/cmd_vel` | Twist | Velocity commands |

**Services:**
- `/motor_power` (SetBool) - Enable/disable motors
- `/sound` (Sound) - Buzzer control
- `/reset` (SetBool) - Hardware reset

**Secondary Node: DiffDriveController**

**Purpose:** Compute odometry from encoders

**Topics Published:**
- `/odom` (Odometry)
- `/tf` (odom → base_footprint)

**Topics Subscribed:**
- `/joint_states` (JointState)
- `/imu` (optional, for heading fusion)

**Key Parameters (burger.yaml):**
```yaml
turtlebot3_node:
  opencr:
    id: 200
    baud_rate: 1000000
    protocol_version: 2.0
  wheels:
    separation: 0.160
    radius: 0.033
  motors:
    profile_acceleration_constant: 214.577

diff_drive_controller:
  odometry:
    publish_tf: true
    use_imu: true
    frame_id: "odom"
    child_frame_id: "base_footprint"
```

**Internal Architecture:**
```cpp
DynamixelSDKWrapper
├── get_data_from_device<T>() - Thread-safe cached read
├── set_data_to_device() - Mutex-protected write
├── read_data_set() - Bulk read from OpenCR (200 bytes)
└── Thread Safety: 3 mutexes (sdk, read_data, write_data)

Control Table (control_table.hpp)
├── 0-10: Device info (model, firmware, ID)
├── 10-19: Timing (millis, micros, heartbeat)
├── 20-30: LEDs, buttons, bumpers
├── 30-100: Sensor readings (IMU, encoder, battery)
└── 120-180: Motor commands and status
```

---

### turtlebot3_bringup (System Layer)

**Purpose:** Orchestrate robot system startup

**Launch Files:**
| Launch | Purpose |
|--------|---------|
| `robot.launch.py` | Main robot startup |
| `turtlebot3_state_publisher.launch.py` | URDF → TF |
| `rviz2.launch.py` | Visualization |
| `camera.launch.py` | Raspberry Pi camera |

**Coordination:**
1. robot_state_publisher (URDF/TF)
2. LIDAR driver (LDS-01/02/03 variant)
3. turtlebot3_node (USB port argument)

---

### turtlebot3_description (Description Layer)

**Purpose:** Robot geometry and visualization

**URDF Structure (Burger):**
```
base_footprint
└── base_link (fixed, 10mm offset)
    ├── wheel_left_link (continuous)
    ├── wheel_right_link (continuous)
    ├── imu_link
    └── base_scan (LIDAR)
```

**Physical Properties:**
- Wheel separation: 0.16m
- Wheel radius: 0.033m
- Total mass: ~0.8kg

---

### turtlebot3_teleop (HCI Layer)

**Purpose:** Manual keyboard control

**Node:** `teleop_keyboard.py`

**Hardware-Aware Limits:**
| Model | Max Linear | Max Angular |
|-------|------------|-------------|
| Burger | 0.22 m/s | 2.84 rad/s |
| Waffle | 0.26 m/s | 1.82 rad/s |

**Control Mapping:**
- W/X: Linear velocity ±0.01 m/s
- A/D: Angular velocity ±0.1 rad/s
- Space/S: Emergency stop

**Publishes:** `/cmd_vel` (Twist)

---

### turtlebot3_example (Application Layer)

**Purpose:** Demonstration applications

**patrol_server (Action Pattern):**
- Subscribes: `/odom`
- Publishes: `/cmd_vel`
- Action: `/patrol` (Patrol.action)
- State machine: Go front → Turn → Validate

**absolute_move (Goal Seeking):**
- Proportional control to waypoints
- Uses odometry feedback

**obstacle_detection (Safety Filter):**
- Subscribes: `/scan`, `/cmd_vel_raw`
- Publishes: `/cmd_vel` (filtered)
- Stops if obstacle < 0.5m

---

### turtlebot3_gazebo (Simulation Layer)

**Purpose:** Physics simulation

**Plugins:**
- `turtlebot3_drive.cpp` - Motor command → wheel forces
- `obstacle*.cpp` - Environmental objects
- `traffic_bar_plugin.cpp` - Dynamic obstacles

**Launch Files:**
- `turtlebot3_world.launch.py` - Standard world
- `multi_robot.launch.py` - Multi-robot with namespacing
- `turtlebot3_dqn_stage*.launch.py` - RL training environments

---

### turtlebot3_fake_node (Testing Layer)

**Purpose:** Kinematic simulation (no physics)

**Node:** `turtlebot3_fake_node.cpp`

**Mechanism:**
- Subscribes: `/cmd_vel`
- Simulates wheel encoder deltas
- Publishes: `/odom`, `/joint_states`, `/tf`

**Parameters:**
- `wheels.separation`: 0.16m
- `wheels.radius`: 0.033m
- Update rate: 100 Hz

---

### turtlebot3_navigation2 (Planning Layer)

**Purpose:** Nav2 stack integration

**Content:**
- Nav2 configuration files
- Map files (pre-saved occupancy grids)
- Hardware-specific parameters

---

### turtlebot3_cartographer (Perception Layer)

**Purpose:** Online 2D SLAM

**Integration:**
- Subscribes: `/scan`, `/odom`, `/tf`
- Publishes: `/map`, TF (map→odom)

---

## 2. System Architecture Analysis

### Data Flow for Motion Control

```
User/Application (teleop, patrol, Nav2)
         ↓
    /cmd_vel (Twist)
         ↓
turtlebot3_node (cmd_vel_callback)
         ↓
DynamixelSDKWrapper::set_data_to_device()
         ↓
USB Serial → OpenCR Board
         ↓
Motor Driver (XL430 servos) → Wheel Rotation
         ↓
Encoder reading (50ms cycle)
         ↓
/joint_states → DiffDriveController
         ↓
Odometry calculation (differential drive)
         ↓
/odom, /tf (odom→base_footprint)
```

### Perception Data Flow

```
Physical Sensors
├── OpenCR IMU → /imu
├── Motor encoders → /joint_states
├── Bumpers → /sensor_state
└── LIDAR → /scan
         ↓
Available to Higher Layers:
├── Navigation2 (uses /odom, /scan, /tf)
├── Cartographer SLAM (uses /scan, /odom, /tf)
└── Examples (use /odom, /scan)
```

### TF Tree

```
map (from SLAM/Navigation2)
  └─ odom (from DiffDriveController)
      └─ base_footprint
          └─ base_link
              ├─ imu_link
              ├─ wheel_left_link
              ├─ wheel_right_link
              └─ base_scan (LIDAR)
```

---

## 3. Coordination Patterns

### Publish-Subscribe (Topic-Based)

**High-Volume Streams:**
- `/joint_states` (20 Hz)
- `/imu` (20 Hz)
- `/scan` (~10-20 Hz)

**Control Loop:**
- `/cmd_vel` (10-100 Hz)
- `/odom` (rate-dependent)

### Request-Reply (Service-Based)

- `/motor_power` - Enable/disable motors
- `/sound` - Buzzer control
- `/reset` - Hardware reset

### Action-Based (Goal-Oriented)

- Patrol action with feedback
- Preemptable goals

### Parameter Event Callbacks

```cpp
parameter_event_callback()
  if (motors.profile_acceleration changed)
    write new acceleration to OpenCR
```

---

## 4. Failure Propagation

### Single Points of Failure

| Component | Impact |
|-----------|--------|
| OpenCR Board | No failover (hardware) |
| USB Cable | No fallback communication |
| turtlebot3_node | All sensors/control stop |
| robot_state_publisher | TF tree breaks |
| LIDAR driver | Obstacle detection fails |

### Failure Modes

| Failure | Detection | Propagation | Recovery |
|---------|-----------|-------------|----------|
| USB disconnect | is_connected_to_device() | Node shutdown | Manual reconnection |
| Encoder failure | Missing data | Stale odom | Continues with last values |
| IMU malfunction | Zeros/NaN | SLAM divergence | None (continues) |
| LIDAR disconnect | No /scan | Navigation blind | May crash into obstacles |

---

## 5. Coupling Analysis

### Tight Coupling

| Coupling | Risk |
|----------|------|
| DynamixelSDKWrapper ↔ OpenCR | Protocol-specific |
| Control table addresses | Firmware version dependent |
| USB port configuration | Hardware-specific |
| Parameter structure | YAML format dependent |

### Loose Coupling

| Component | Why Loose |
|-----------|-----------|
| Topic-based communication | Multiple producers/consumers |
| URDF geometry | Decoupled from control logic |
| Example applications | Only depend on cmd_vel, odom |

---

## 6. Reusability Analysis

### High Reusability

| Component | Score | Reason |
|-----------|-------|--------|
| Odometry calculation | 9/10 | Standard differential drive |
| Teleop keyboard | 8/10 | Works with any cmd_vel robot |
| DiffDriveController | 9/10 | Generic 2-wheel control |
| TF publishing | 8/10 | Standard ROS pattern |

### Low Reusability

| Component | Score | Reason |
|-----------|-------|--------|
| DynamixelSDKWrapper | 3/10 | OpenCR + XL430 specific |
| Control table | 2/10 | Firmware-specific addresses |
| Gazebo plugins | 4/10 | Tuned for TurtleBot3 physics |

---

## 7. Topic Summary

| Topic | Publisher | Subscriber | Type |
|-------|-----------|------------|------|
| `/cmd_vel` | teleop/Nav2/examples | turtlebot3_node | Twist |
| `/odom` | DiffDriveController | Nav2/SLAM | Odometry |
| `/joint_states` | turtlebot3_node | DiffDriveController | JointState |
| `/imu` | turtlebot3_node | Odometry (optional) | Imu |
| `/scan` | LIDAR driver | Nav2/SLAM/examples | LaserScan |
| `/battery_state` | turtlebot3_node | Monitor | BatteryState |
| `/sensor_state` | turtlebot3_node | Applications | SensorState |
| `/tf` | DiffDriveController, SLAM | All | TFMessage |
| `/map` | Cartographer | Navigation | OccupancyGrid |

---

## 8. Parameter Summary

| Node | Key Parameters |
|------|----------------|
| turtlebot3_node | `opencr.id`, `opencr.baud_rate`, `wheels.separation`, `wheels.radius` |
| diff_drive_controller | `publish_tf`, `use_imu`, `frame_id`, `child_frame_id` |
| teleop_keyboard | Max velocities (from TURTLEBOT3_MODEL env) |
| Navigation2 | Planner, controller, costmap configs |

---

## 9. Integration Complexity Hotspots

| Hotspot | Issue | Mitigation |
|---------|-------|------------|
| OpenCR protocol | Hardcoded register addresses | Detect firmware version |
| USB connection | No reconnection logic | Manual restart |
| Sensor failures | Stale data published silently | Add diagnostics |
| Multi-robot | Namespace coordination | Launch arguments |

---

## 10. Architecture Insights

**Strengths:**
- Clear separation of concerns (hardware → control → perception → planning)
- Standards compliance (ROS2 messages, services, actions)
- Multiple testing alternatives (fake_node, Gazebo)
- Namespace support for multi-robot

**Weaknesses:**
- Monolithic turtlebot3_node (single point of failure)
- Tight hardware coupling (DynamixelSDKWrapper)
- No watchdog/health monitoring
- Limited error handling

**Recommendations:**
1. Decompose turtlebot3_node into smaller focused nodes
2. Add hardware abstraction interface
3. Implement reconnection logic
4. Add diagnostic health monitoring

**Suitable For:**
- Education and research
- ROS2 learning platform
- Multi-robot experiments
- Navigation algorithm development

