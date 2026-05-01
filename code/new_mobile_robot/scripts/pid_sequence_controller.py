#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped


class PID:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        self.integral = max(min(self.integral, 1.0), -1.0)

        if self.first:
            derivative = 0.0
            self.first = False
        else:
            derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, self.output_limit), -self.output_limit)

        self.prev_error = error
        return output


class PIDSequenceController(Node):
    def __init__(self):
        super().__init__('pid_sequence_controller')

        self.move_distance = 1.0
        self.turn_angle = math.pi / 2.0

        self.linear_pid = PID(kp=0.65, ki=0.0, kd=0.08, output_limit=0.16)
        self.angular_pid = PID(kp=1.8, ki=0.0, kd=0.18, output_limit=0.75)
        self.heading_hold_pid = PID(kp=1.2, ki=0.0, kd=0.05, output_limit=0.45)

        self.x = None
        self.y = None
        self.yaw = None

        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.target_yaw = None

        self.prev_time = None

        self.states = [
            "FORWARD_1M",
            "BACKWARD_1M",
            "TURN_LEFT_90",
            "FORWARD_1M_AFTER_LEFT",
            "TURN_RIGHT_90",
            "DONE"
        ]

        self.state_index = 0
        self.current_state = self.states[self.state_index]

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/pid_sequence_path', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("PID Sequence Controller started")
        self.get_logger().info("Sequence: forward 1m -> backward 1m -> left 90 -> forward 1m -> right 90 -> stop")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def reset_state_reference(self):
        self.start_x = self.x
        self.start_y = self.y
        self.start_yaw = self.yaw

        self.linear_pid.reset()
        self.angular_pid.reset()
        self.heading_hold_pid.reset()

        self.get_logger().info(f"Starting state: {self.current_state}")

    def next_state(self):
        self.stop_robot()

        self.state_index += 1
        self.current_state = self.states[self.state_index]

        self.reset_state_reference()

        if self.current_state == "TURN_LEFT_90":
            self.target_yaw = self.normalize_angle(self.start_yaw + self.turn_angle)

        elif self.current_state == "TURN_RIGHT_90":
            self.target_yaw = self.normalize_angle(self.start_yaw - self.turn_angle)

        self.get_logger().info(f"Switched to: {self.current_state}")

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

        if len(self.path_msg.poses) > 1500:
            self.path_msg.poses.pop(0)

        self.path_pub.publish(self.path_msg)

    def distance_along_start_heading(self):
        dx = self.x - self.start_x
        dy = self.y - self.start_y
        return dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)

    def move_straight_pid(self, direction, dt):
        travelled = direction * self.distance_along_start_heading()
        distance_error = self.move_distance - travelled

        cmd = Twist()

        if distance_error < 0.03:
            self.next_state()
            return

        linear_output = self.linear_pid.compute(distance_error, dt)
        cmd.linear.x = direction * abs(linear_output)

        heading_error = self.normalize_angle(self.start_yaw - self.yaw)
        cmd.angular.z = self.heading_hold_pid.compute(heading_error, dt)

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"{self.current_state}: travelled={travelled:.2f}, error={distance_error:.2f}, "
            f"linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}",
            throttle_duration_sec=0.8
        )

    def turn_pid(self, dt):
        yaw_error = self.normalize_angle(self.target_yaw - self.yaw)

        cmd = Twist()

        if abs(yaw_error) < 0.035:
            self.next_state()
            return

        cmd.linear.x = 0.0
        cmd.angular.z = self.angular_pid.compute(yaw_error, dt)

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"{self.current_state}: yaw_error={math.degrees(yaw_error):.1f} deg, "
            f"angular={cmd.angular.z:.2f}",
            throttle_duration_sec=0.8
        )

    def control_loop(self):
        if self.x is None or self.yaw is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if self.prev_time is None:
            self.prev_time = now
            self.reset_state_reference()
            return

        dt = now - self.prev_time
        self.prev_time = now

        self.publish_path()

        if self.current_state == "FORWARD_1M":
            self.move_straight_pid(direction=1.0, dt=dt)

        elif self.current_state == "BACKWARD_1M":
            self.move_straight_pid(direction=-1.0, dt=dt)

        elif self.current_state == "TURN_LEFT_90":
            self.turn_pid(dt)

        elif self.current_state == "FORWARD_1M_AFTER_LEFT":
            self.move_straight_pid(direction=1.0, dt=dt)

        elif self.current_state == "TURN_RIGHT_90":
            self.turn_pid(dt)

        elif self.current_state == "DONE":
            self.stop_robot()
            self.get_logger().info("Sequence completed. Robot stopped.", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = PIDSequenceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    try:
        node.stop_robot()
    except Exception:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
