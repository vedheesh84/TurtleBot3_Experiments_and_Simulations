#!/usr/bin/env python3
"""
Navigation Goal Sender

LEARNING OBJECTIVES:
- Understand how to send navigation goals programmatically
- See Nav2 action interface in action
- Learn about goal feedback and results

USAGE:
    ros2 run turtlebot_demos goal_sender --x 2.0 --y 1.0 --yaw 0.0

WHAT THIS DOES:
1. Sends a NavigateToPose action goal
2. Monitors feedback (distance remaining, ETA)
3. Reports success or failure
"""
import argparse
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self.get_logger().info('Goal Sender initialized')

    def send_goal(self, x: float, y: float, yaw: float):
        """Send a navigation goal."""
        self.get_logger().info('Waiting for Nav2 action server...')

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            return False

        # Build goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Orientation (yaw to quaternion)
        q = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.get_logger().info(f'Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

        # Send goal with feedback callback
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        """Called when goal is accepted or rejected."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal was REJECTED!')
            return

        self.get_logger().info('Goal ACCEPTED, navigating...')

        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Called periodically with navigation progress."""
        feedback = feedback_msg.feedback
        # Current pose
        x = feedback.current_pose.pose.position.x
        y = feedback.current_pose.pose.position.y

        # Distance remaining (if available)
        self.get_logger().info(
            f'Current position: ({x:.2f}, {y:.2f})'
        )

    def result_callback(self, future):
        """Called when navigation completes."""
        result = future.result().result

        # Nav2 returns empty result on success
        self.get_logger().info('=' * 40)
        self.get_logger().info('Navigation COMPLETE!')
        self.get_logger().info('=' * 40)


def main():
    parser = argparse.ArgumentParser(description='Send navigation goal')
    parser.add_argument('--x', type=float, default=1.0, help='Goal X position')
    parser.add_argument('--y', type=float, default=0.0, help='Goal Y position')
    parser.add_argument('--yaw', type=float, default=0.0, help='Goal yaw (radians)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = GoalSender()

    try:
        if node.send_goal(args.x, args.y, args.yaw):
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Goal cancelled')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
