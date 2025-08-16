#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, Bool, Int32
from geometry_msgs.msg import PointStamped
import numpy as np
import cv2
import os
from rclpy.executors import MultiThreadedExecutor

class MapCoverageNode(Node):
    def __init__(self):
        super().__init__('map_coverage_node')
        
        # Parameters
        self.declare_parameter('coverage_threshold', 0.95)  # 95% coverage
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('world_name', 'forest_world')
        self.declare_parameter('map_save_path', 'fire_map.png')
        
        self.coverage_threshold = self.get_parameter('coverage_threshold').value
        self.map_topic = self.get_parameter('map_topic').value
        self.world_name = self.get_parameter('world_name').value
        self.map_save_path = self.get_parameter('map_save_path').value

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.all_done_sub = self.create_subscription(
            Bool, '/all_fires_suppressed', self.all_done_callback, 10)
        self.suppressed_fire_sub = self.create_subscription(
            PointStamped, '/suppressed_fire_position', self.suppressed_fire_callback, 10)
        self.suppressed_count_sub = self.create_subscription(
            Int32, '/suppressed_fire_count', self.suppressed_count_callback, 10)

        # Publishers
        self.coverage_pub = self.create_publisher(Float32, '/map_coverage', 10)

        # State
        self.current_map = None
        self.suppressed_fires = []
        self.suppressed_count = 0

        self.get_logger().info("🚀 Map Coverage Node initialized")

    def map_callback(self, msg):
        self.current_map = msg
        coverage = self.compute_coverage()
        self.coverage_pub.publish(Float32(data=coverage))
        if self.all_suppressed and coverage >= self.coverage_threshold:
            self.save_map_with_fires()

    def suppressed_fire_callback(self, msg):
        self.suppressed_fires.append((msg.point.x, msg.point.y))

    def suppressed_count_callback(self, msg):
        self.suppressed_count = msg.data

    def all_done_callback(self, msg):
        self.all_suppressed = msg.data
        if self.all_suppressed and self.current_map is not None:
            coverage = self.compute_coverage()
            if coverage >= self.coverage_threshold:
                self.save_map_with_fires()

    def compute_coverage(self):
        if self.current_map is None:
            return 0.0
        data = np.array(self.current_map.data).reshape(
            self.current_map.info.height, self.current_map.info.width)
        total_cells = data.size
        unknown_cells = np.sum(data == -1)
        known_cells = total_cells - unknown_cells
        return known_cells / total_cells if total_cells > 0 else 0.0

    def save_map_with_fires(self):
        if self.current_map is None:
            self.get_logger().warning("No map available to save")
            return
        
        # Convert occupancy grid to image
        data = np.array(self.current_map.data).reshape(
            self.current_map.info.height, self.current_map.info.width)
        image = np.zeros((self.current_map.info.height, self.current_map.info.width, 3), dtype=np.uint8)
        
        # Map values: -1 (unknown) -> gray, 0 (free) -> white, 100 (occupied) -> black
        image[data == -1] = [128, 128, 128]  # Gray for unknown
        image[data == 0] = [255, 255, 255]   # White for free
        image[data == 100] = [0, 0, 0]       # Black for occupied

        # Convert fire positions to map coordinates
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y

        def world_to_map(x, y):
            mx = int((x - origin_x) / resolution)
            my = int((y - origin_y) / resolution)
            return mx, my

        # Draw suppressed fires (blue)
        for pos in self.suppressed_fires:
            mx, my = world_to_map(pos[0], pos[1])
            if 0 <= mx < self.current_map.info.width and 0 <= my < self.current_map.info.height:
                cv2.circle(image, (mx, my), 5, (255, 0, 0), -1)  # Blue dot

        # Save image
        cv2.imwrite(self.map_save_path, image)
        self.get_logger().info(f"Map saved to {self.map_save_path} with {self.suppressed_count} suppressed fires")

def main(args=None):
    rclpy.init(args=args)
    node = MapCoverageNode()
    executor = MultiThreadedExecutor()
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()