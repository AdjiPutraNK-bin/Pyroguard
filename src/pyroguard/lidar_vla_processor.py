#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

class LidarVlaProcessorNode(Node):
    def __init__(self):
        super().__init__('lidar_vla_processor_node')
        
        # Parameters
        self.declare_parameter('min_safe_distance', 0.3)  # Minimum distance to obstacles
        self.min_safe_distance = self.get_parameter('min_safe_distance').value

        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, '/world/forest_world/model/turtlebot4/link/lidar_link/sensor/lidar/scan', self.lidar_callback, 10)
        self.vla_sub = self.create_subscription(
            Float32MultiArray, '/vla_output', self.vla_callback, 10)
        
        # Publisher
        self.obs_pub = self.create_publisher(Float32MultiArray, '/obs', 10)

        # State
        self.vla_data = None  # [distance_to_fire, fire_size]
        self.lidar_min_distance = float('inf')

        # Timer for publishing observations
        self.timer = self.create_timer(0.1, self.publish_observation)
        
        self.get_logger().info("🚀 LIDAR-VLA Processor Node initialized")

    def lidar_callback(self, msg):
        # Compute minimum obstacle distance from LIDAR scan
        ranges = np.array(msg.ranges)
        ranges = ranges[np.isfinite(ranges)]  # Filter out inf/nan
        self.lidar_min_distance = float(np.min(ranges)) if len(ranges) > 0 else float('inf')

    def vla_callback(self, msg):
        # Store VLA data
        if len(msg.data) == 2:
            self.vla_data = np.array(msg.data, dtype=np.float32)
        else:
            self.get_logger().error("Invalid VLA data: expected 2 elements")

    def publish_observation(self):
        if self.vla_data is not None and self.lidar_min_distance != float('inf'):
            # Combine VLA and LIDAR data
            obs = np.concatenate([self.vla_data, [self.lidar_min_distance]])
            msg = Float32MultiArray()
            msg.data = obs.tolist()
            self.obs_pub.publish(msg)
        else:
            if self.vla_data is None and self.lidar_min_distance == float('inf'):
                self.get_logger().warning("Incomplete observation data: missing both VLA and LIDAR")
            elif self.vla_data is None:
                self.get_logger().warning("Incomplete observation data: missing VLA data")
            elif self.lidar_min_distance == float('inf'):
                self.get_logger().warning("Incomplete observation data: missing LIDAR data")

def main(args=None):
    rclpy.init(args=args)
    node = LidarVlaProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()