#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class CenterToRoomNav(Node):
    def __init__(self):
        super().__init__('center_to_room_nav')

        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Safer doorway path:
        # small center movements -> align at doorway -> enter slightly inside
        self.waypoints = [
            (0.20, 0.00, 0.00),
            (0.40, 0.00, 0.00),
            (0.55, 0.20, 0.40),
            (0.70, 0.40, 0.70),
            (0.90, 0.60, 0.90),
        ]

        self.index = 0

        self.get_logger().info("Safe room-entry Nav2 demo started")
        self.get_logger().info("Waiting for Nav2 action server...")
        self.client.wait_for_server()
        self.get_logger().info("Nav2 action server connected")

        self.send_next_goal()

    def yaw_to_quaternion(self, yaw):
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def send_next_goal(self):
        if self.index >= len(self.waypoints):
            self.get_logger().info("Robot entered doorway/room area. Demo finished.")
            rclpy.shutdown()
            return

        x, y, yaw = self.waypoints[self.index]

        goal = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal.pose = pose

        self.get_logger().info(
            f"Sending waypoint {self.index + 1}/{len(self.waypoints)}: "
            f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected. Skipping to next waypoint.")
            self.index += 1
            time.sleep(1.0)
            self.send_next_goal()
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, msg):
        self.get_logger().info(
            f"Distance remaining: {msg.feedback.distance_remaining:.2f} m",
            throttle_duration_sec=2.0
        )

    def result_callback(self, future):
        result = future.result()
        self.get_logger().info(
            f"Waypoint {self.index + 1} finished with status: {result.status}"
        )

        self.index += 1
        time.sleep(1.0)
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = CenterToRoomNav()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
