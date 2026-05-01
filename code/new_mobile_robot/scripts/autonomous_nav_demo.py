#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class AutonomousNavDemo(Node):
    def __init__(self):
        super().__init__('autonomous_nav_demo')

        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Waypoints in map frame.
        # These are safe demo points around the central apartment area.
        # If one goal fails, adjust x/y slightly.
        self.waypoints = [
            (0.8, 0.0, 0.0),
            (1.2, 0.8, 1.57),
            (0.2, 1.2, 3.14),
            (-0.8, 0.8, -1.57),
            (-0.8, -0.4, 0.0),
            (0.4, -0.8, 1.57),
            (0.0, 0.0, 0.0),
        ]

        self.current_index = 0

        self.get_logger().info('Autonomous Nav2 demo started.')
        self.get_logger().info('Waiting for NavigateToPose action server...')

        self.client.wait_for_server()

        self.get_logger().info('Nav2 action server connected.')
        self.send_next_goal()

    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('All waypoints completed. Demo finished.')
            rclpy.shutdown()
            return

        x, y, yaw = self.waypoints[self.current_index]

        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg.pose = pose

        self.get_logger().info(
            f'Sending goal {self.current_index + 1}/{len(self.waypoints)}: '
            f'x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )

        send_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected. Moving to next waypoint.')
            self.current_index += 1
            time.sleep(1.0)
            self.send_next_goal()
            return

        self.get_logger().info('Goal accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining

        self.get_logger().info(
            f'Distance remaining: {distance:.2f} m',
            throttle_duration_sec=2.0
        )

    def result_callback(self, future):
        result = future.result()

        self.get_logger().info(
            f'Goal {self.current_index + 1} finished with status: {result.status}'
        )

        self.current_index += 1

        # Small pause between goals
        time.sleep(1.0)

        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
