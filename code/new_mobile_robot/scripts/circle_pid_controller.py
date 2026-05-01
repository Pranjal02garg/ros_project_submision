#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped


class CirclePIDController(Node):
    def __init__(self):
        super().__init__('circle_pid_controller')

        # Smaller circle so robot does not hit walls
        self.radius = 0.45
        self.linear_speed = 0.06
        self.direction = 1.0   # 1 = counter-clockwise, -1 = clockwise

        # PID gains for radial correction
        self.kp = 1.8
        self.ki = 0.0
        self.kd = 0.20

        # Heading correction gain
        self.kp_heading = 1.0

        # Speed limits
        self.max_linear = 0.08
        self.max_angular = 0.70

        self.x = None
        self.y = None
        self.yaw = None

        self.start_x = None
        self.start_y = None
        self.center_x = None
        self.center_y = None

        self.prev_error = 0.0
        self.integral_error = 0.0
        self.prev_time = None

        self.total_angle_travelled = 0.0
        self.prev_angle_from_center = None
        self.completed_circle = False

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/circle_pid_path', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Safe Circle PID Controller started')
        self.get_logger().info(f'Circle radius: {self.radius} m')
        self.get_logger().info(f'Forward speed: {self.linear_speed} m/s')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.center_x is None:
            self.start_x = self.x
            self.start_y = self.y

            # Put circle center to the left side of robot
            self.center_x = self.x - self.direction * self.radius * math.sin(self.yaw)
            self.center_y = self.y + self.direction * self.radius * math.cos(self.yaw)

            dx = self.x - self.center_x
            dy = self.y - self.center_y
            self.prev_angle_from_center = math.atan2(dy, dx)

            self.get_logger().info(
                f'Circle center: x={self.center_x:.2f}, y={self.center_y:.2f}'
            )

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def publish_path(self):
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.02
        pose.pose.orientation.w = 1.0

        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(pose)

        self.path_pub.publish(self.path_msg)

    def update_circle_progress(self, angle_from_center):
        delta = self.normalize_angle(angle_from_center - self.prev_angle_from_center)

        if self.direction > 0 and delta < -math.pi / 2:
            delta += 2.0 * math.pi
        elif self.direction < 0 and delta > math.pi / 2:
            delta -= 2.0 * math.pi

        self.total_angle_travelled += abs(delta)
        self.prev_angle_from_center = angle_from_center

    def control_loop(self):
        if self.x is None or self.center_x is None:
            return

        if self.completed_circle:
            self.stop_robot()
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if self.prev_time is None:
            self.prev_time = now
            return

        dt = now - self.prev_time
        if dt <= 0.0:
            return

        dx = self.x - self.center_x
        dy = self.y - self.center_y

        distance_from_center = math.sqrt(dx * dx + dy * dy)
        angle_from_center = math.atan2(dy, dx)

        self.update_circle_progress(angle_from_center)

        # Stop after one full circle
        if self.total_angle_travelled >= 2.0 * math.pi:
            self.completed_circle = True
            self.stop_robot()
            self.get_logger().info('One full circle completed. Robot stopped.')
            return

        radial_error = distance_from_center - self.radius

        self.integral_error += radial_error * dt
        self.integral_error = max(min(self.integral_error, 0.5), -0.5)

        derivative_error = (radial_error - self.prev_error) / dt

        pid_output = (
            self.kp * radial_error +
            self.ki * self.integral_error +
            self.kd * derivative_error
        )

        # Desired heading is tangent to the circle
        if self.direction > 0:
            desired_yaw = angle_from_center + math.pi / 2.0
        else:
            desired_yaw = angle_from_center - math.pi / 2.0

        heading_error = self.normalize_angle(desired_yaw - self.yaw)

        # Base angular velocity for a circle
        base_angular = self.direction * (self.linear_speed / self.radius)

        angular_cmd = (
            base_angular +
            self.direction * pid_output +
            self.kp_heading * heading_error
        )

        linear_cmd = self.linear_speed

        linear_cmd = max(min(linear_cmd, self.max_linear), -self.max_linear)
        angular_cmd = max(min(angular_cmd, self.max_angular), -self.max_angular)

        cmd = Twist()
        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd
        self.cmd_pub.publish(cmd)

        self.prev_error = radial_error
        self.prev_time = now

        self.publish_path()

        progress_deg = math.degrees(self.total_angle_travelled)

        self.get_logger().info(
            f'progress={progress_deg:.1f} deg, radial_error={radial_error:.3f}, '
            f'linear={linear_cmd:.2f}, angular={angular_cmd:.2f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = CirclePIDController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
