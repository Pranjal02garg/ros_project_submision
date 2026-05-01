#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class SimplePController(Node):
    def __init__(self):
        super().__init__('simple_p_controller')

        # Goal in odom frame
        self.goal_x = 1.0
        self.goal_y = 0.0

        # P gains
        self.kp_linear = 0.4
        self.kp_angular = 1.2

        # Safety speed limits
        self.max_linear = 0.18
        self.max_angular = 0.8

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Simple P Controller started')
        self.get_logger().info(f'Goal: x={self.goal_x}, y={self.goal_y}')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # Quaternion to yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if self.current_x is None:
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y

        distance_error = math.sqrt(dx * dx + dy * dy)
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - self.current_yaw)

        cmd = Twist()

        # Stop if close to goal
        if distance_error < 0.05:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info('Goal reached. Robot stopped.')
            return

        # Rotate first if heading error is large
        if abs(yaw_error) > 0.25:
            cmd.linear.x = 0.0
            cmd.angular.z = self.kp_angular * yaw_error
        else:
            cmd.linear.x = self.kp_linear * distance_error
            cmd.angular.z = self.kp_angular * yaw_error

        # Limit speeds
        cmd.linear.x = max(min(cmd.linear.x, self.max_linear), -self.max_linear)
        cmd.angular.z = max(min(cmd.angular.z, self.max_angular), -self.max_angular)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimplePController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    stop = Twist()
    node.cmd_pub.publish(stop)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
