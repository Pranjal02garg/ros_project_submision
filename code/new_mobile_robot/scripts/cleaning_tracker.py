#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

import tf2_ros


class CleaningTracker(Node):
    def __init__(self):
        super().__init__('cleaning_tracker')

        # Map/coverage settings
        self.map_frame = 'map'
        self.robot_frame = 'base_footprint'

        self.resolution = 0.05          # 5 cm per cell
        self.width_m = 16.0             # world width in meters
        self.height_m = 12.0            # world height in meters
        self.cleaning_radius = 0.22      # robot cleaning footprint radius

        self.width = int(self.width_m / self.resolution)
        self.height = int(self.height_m / self.resolution)

        self.origin_x = -self.width_m / 2.0
        self.origin_y = -self.height_m / 2.0

        # -1 = unknown/uncovered, 100 = cleaned
        self.grid = [-1] * (self.width * self.height)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame

        self.last_x = None
        self.last_y = None
        self.total_cells = self.width * self.height

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.grid_pub = self.create_publisher(OccupancyGrid, '/coverage_grid', 10)
        self.path_pub = self.create_publisher(Path, '/cleaned_path', 10)
        self.percent_pub = self.create_publisher(Float32, '/coverage_percent', 10)

        self.timer = self.create_timer(0.2, self.update_tracking)

        self.get_logger().info('Cleaning tracker started.')
        self.get_logger().info('Publishing /coverage_grid, /cleaned_path, /coverage_percent')

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def grid_index(self, gx, gy):
        return gy * self.width + gx

    def mark_cleaned_area(self, x, y):
        center_gx, center_gy = self.world_to_grid(x, y)
        radius_cells = int(self.cleaning_radius / self.resolution)

        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= radius_cells * radius_cells:
                    gx = center_gx + dx
                    gy = center_gy + dy

                    if 0 <= gx < self.width and 0 <= gy < self.height:
                        idx = self.grid_index(gx, gy)
                        self.grid[idx] = 100

    def add_path_point(self, x, y):
        if self.last_x is not None:
            dist = math.sqrt((x - self.last_x) ** 2 + (y - self.last_y) ** 2)
            if dist < 0.05:
                return

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.02
        pose.pose.orientation.w = 1.0

        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(pose)

        self.last_x = x
        self.last_y = y

    def publish_coverage_grid(self):
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid

        self.grid_pub.publish(msg)

    def publish_coverage_percent(self):
        cleaned = sum(1 for cell in self.grid if cell == 100)
        percent = (cleaned / self.total_cells) * 100.0

        msg = Float32()
        msg.data = percent
        self.percent_pub.publish(msg)

    def update_tracking(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            self.mark_cleaned_area(x, y)
            self.add_path_point(x, y)

            self.publish_coverage_grid()
            self.path_pub.publish(self.path_msg)
            self.publish_coverage_percent()

        except Exception as e:
            self.get_logger().warn(f'Waiting for TF map -> base_footprint: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = CleaningTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
