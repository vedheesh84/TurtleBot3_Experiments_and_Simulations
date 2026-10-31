# TurtleBot3 Capstone Demonstration Guide

**Level 1 Capstone Proving Ground: Articles 1–25 in Action**

This guide provides a structured 6-stage walkthrough demonstrating how the core concepts taught in Articles 1–25 (Nodes, Topics, Services, Actions, Parameters, Launch Design, Namespacing, Lifecycle, TF Trees, Simulation, Debugging, and Systems Thinking) operate together inside an embodied Autonomous Mobile Robot (AMR).

---

## Prerequisites

```bash
cd "02 — Domains/ROS2/AMR_ws/turtlebot3_ws"
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## Stage 1: Kinematic Testing & Basic Velocity Control (Articles A0–A3b, B1)

*Focus: Minimal node execution, asynchronous topic streaming (`/cmd_vel`), odometry feedback, and lightweight kinematic simulation without GPU/Gazebo physics.*

1. **Launch the Kinematic Simulator**:
   ```bash
   ros2 launch turtlebot3_fake_node turtlebot3_fake_node.launch.py
   ```
2. **Drive the Robot via Teleop**:
   In a second terminal:
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```
3. **Verify Topics & Frequency**:
   ```bash
   ros2 topic echo /cmd_vel
   ros2 topic hz /odom
   ```

---

## Stage 2: Spatial Frames & TF Tree Verification (Articles E1, E2)

*Focus: Coordinate frames, spatial transformations, parent-child relationships, and TF tree inspection.*

1. **Keep the robot running from Stage 1.**
2. **Echo Spatial Transforms**:
   ```bash
   ros2 run tf2_ros tf2_echo odom base_footprint
   ros2 run tf2_ros tf2_echo base_link base_scan
   ```
3. **Generate the TF Tree Diagram**:
   ```bash
   ros2 run tf2_tools view_frames
   # Inspect generated frames.pdf (odom -> base_footprint -> base_link -> base_scan/imu_link/wheels)
   ```

---

## Stage 3: ROS2 Actions in Practice (Articles B3, B3b, B4a)

*Focus: Long-running goal execution, continuous state feedback loops, and cancellation handling using `Patrol.action`.*

1. **Start the Patrol Action Server**:
   ```bash
   ros2 run turtlebot3_example turtlebot3_patrol_server
   ```
2. **Send a Square Patrol Goal via Action Client**:
   In another terminal:
   ```bash
   ros2 run turtlebot3_example turtlebot3_patrol_client
   ```
3. **Monitor Action Goals & Feedback**:
   ```bash
   ros2 action list
   ros2 action info /patrol
   ```
   Observe the robot completing sequential waypoints with real-time feedback updates rather than blocking synchronously.

---

## Stage 4: Multi-Robot Namespacing & Scaling (Articles C1, C2)

*Focus: Multi-robot instantiation, `PushRosNamespace`, topic remapping, and scaling without data collisions.*

1. **Launch Multi-Robot Simulation**:
   ```bash
   ros2 launch turtlebot3_gazebo multi_robot.launch.py
   ```
2. **Verify Namespace Isolation**:
   In another terminal:
   ```bash
   ros2 node list
   # Output:
   # /tb3_0/robot_state_publisher
   # /tb3_0/turtlebot3_node
   # /tb3_1/robot_state_publisher
   # /tb3_1/turtlebot3_node

   ros2 topic list
   # Output:
   # /tb3_0/cmd_vel, /tb3_0/scan, /tb3_0/odom
   # /tb3_1/cmd_vel, /tb3_1/scan, /tb3_1/odom
   ```
3. **Command Individual Namespaces**:
   ```bash
   ros2 topic pub /tb3_0/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
   ros2 topic pub /tb3_1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.2}}" --once
   ```

---

## Stage 5: Safety Filters & Graph Debugging (Articles D2, G1, G2)

*Focus: Systematic introspection, topic interception, and safety filtering.*

1. **Launch Gazebo Simulation World**:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```
2. **Run Obstacle Detection Safety Node**:
   ```bash
   ros2 run turtlebot3_example turtlebot3_obstacle_detection
   ```
3. **Drive Toward an Obstacle**:
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```
   Observe the safety node intercepting `/cmd_vel` and enforcing a hard stop when LiDAR `/scan` detects an obstacle within 0.5 meters.
4. **Introspect Graph Health**:
   ```bash
   rqt_graph
   ros2 doctor --report
   ```

---

## Stage 6: The AMR Gateway — SLAM & Autonomous Navigation (Articles H1 & Next Steps)

*Focus: Capstone synthesis, 2D LiDAR SLAM mapping with Cartographer, and Nav2 autonomous path planning.*

1. **Run 2D Cartographer SLAM**:
   ```bash
   ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
   ```
2. **Drive to Map the Environment**:
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```
3. **Save the Occupancy Grid Map**:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/my_capstone_map
   ```
4. **Launch Navigation2 Autonomous Goal Seeking**:
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=~/my_capstone_map.yaml use_sim_time:=true
   ```
   Use RViz2's "Nav2 Goal" button to command the robot to navigate autonomously through obstacles.

---

## Conclusion & Transition to Specialized Kits

You have successfully validated all 25 core curriculum concepts on an embodied mobile robot. 

From here, you are ready to advance to:
- **Dedicated AMR & Nav2 Curriculum**: Deep-diving into Costmap inflation layers, EKF sensor fusion, and Behavior Trees.
- **Robotic Arm & Manipulation**: Kinematics, trajectory planning, and MoveIt2.
- **Advanced Embodiments**: Legged Locomotion (Quadruped), Marine Systems (Reef Drone), and Social Robotics (Companion Head).
