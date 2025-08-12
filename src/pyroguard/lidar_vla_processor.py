#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
import numpy as np
import math
import re

class LidarVlaProcessorNode(Node):
    def __init__(self):
        super().__init__('lidar_vla_processor_node')
        
        # Parameters
        self.declare_parameter('min_safe_distance', 0.3)
        self.declare_parameter('lidar_topic', '/world/forest_world/model/turtlebot4/link/lidar_link/sensor/lidar/scan')
        self.declare_parameter('world_name', 'forest_world')
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.world_name = self.get_parameter('world_name').value

        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, self.lidar_topic, self.lidar_callback, 10)
        self.vla_sub = self.create_subscription(
            Float32MultiArray, '/vla_output', self.vla_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.fire_pose_sub = self.create_subscription(
            TFMessage, f'/world/{self.world_name}/pose/info', self.fire_pose_callback, 10)
        self.suppressed_fire_sub = self.create_subscription(
            PointStamped, '/suppressed_fire_position', self.suppressed_fire_callback, 10)

        # Publisher
        self.obs_pub = self.create_publisher(Float32MultiArray, '/obs', 10)

        # State
        self.vla_data = None  # [fire_or_no, fire_size]
        self.lidar_min_distance = float('inf')
        self.robot_pose = None
        self.fire_poses = {}  # model_name: (x, y, z)
        self.suppressed_fires = []  # List of (x, y)

        # Timer for publishing observations
        self.timer = self.create_timer(0.1, self.publish_observation)
        
        self.get_logger().info("🚀 LIDAR-VLA Processor Node initialized")

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = ranges[np.isfinite(ranges)]
        self.lidar_min_distance = float(np.min(ranges)) if len(ranges) > 0 else float('inf')

    def vla_callback(self, msg):
        if len(msg.data) == 2:
            self.vla_data = np.array(msg.data, dtype=np.float32)
        else:
            self.get_logger().error(f"Invalid VLA data: expected 2 elements, got {len(msg.data)}")

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def suppressed_fire_callback(self, msg):
        self.suppressed_fires.append((msg.point.x, msg.point.y))
        self.get_logger().info(f"Added suppressed fire at ({msg.point.x:.2f}, {msg.point.y:.2f})")

    def fire_pose_callback(self, msg):
        self.fire_poses.clear()
        for transform in msg.transforms:
            frame_id = transform.child_frame_id
            if "fire" in frame_id.lower() or "light" in frame_id.lower():
                model_name = self.resolve_model_name(frame_id)
                if model_name:
                    t = transform.transform.translation
                    # Check if fire is suppressed
                    for sx, sy in self.suppressed_fires:
                        if math.hypot(t.x - sx, t.y - sy) < 0.5:  # Threshold for matching
                            break
                    else:
                        self.fire_poses[model_name] = (t.x, t.y, t.z)

    def resolve_model_name(self, frame_id):
        MODEL_PREFIX = "fire_model_"
        if re.match(fr"^{MODEL_PREFIX}\d+$", frame_id):
            return frame_id
        if '::' in frame_id:
            parts = frame_id.split('::')
            if re.match(fr"^{MODEL_PREFIX}\d+$", parts[0]):
                return parts[0]
        match = re.search(rf'({MODEL_PREFIX}\d+)', frame_id)
        if match:
            return match.group(1)
        return None

    def compute_angle_to_fire(self):
        if not self.fire_poses or self.robot_pose is None:
            self.get_logger().warning("Missing robot or fire poses, using angle_to_fire=0.0", throttle_duration_sec=5.0)
            return 0.0
        rx, ry = self.robot_pose.position.x, self.robot_pose.position.y
        yaw = self.quaternion_to_yaw(self.robot_pose.orientation)
        min_dist = float('inf')
        target_angle = 0.0
        for pos in self.fire_poses.values():
            fx, fy = pos[0], pos[1]
            dist = math.hypot(fx - rx, fy - ry)
            if dist < min_dist:
                min_dist = dist
                angle = math.atan2(fy - ry, fx - rx) - yaw
                target_angle = np.arctan2(np.sin(angle), np.cos(angle))
        return target_angle

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_observation(self):
        if self.vla_data is not None and self.lidar_min_distance != float('inf'):
            angle_to_fire = self.compute_angle_to_fire()
            obs = np.concatenate([self.vla_data, [self.lidar_min_distance, angle_to_fire]])
            msg = Float32MultiArray()
            msg.data = obs.tolist()
            self.obs_pub.publish(msg)
        else:
            if self.vla_data is None and self.lidar_min_distance == float('inf'):
                self.get_logger().warning(
                    "Incomplete observation data: missing both VLA and LIDAR",
                    throttle_duration_sec=5.0
                )
            elif self.vla_data is None:
                self.get_logger().warning(
                    "Incomplete observation data: missing VLA data",
                    throttle_duration_sec=5.0
                )
            elif self.lidar_min_distance == float('inf'):
                self.get_logger().warning(
                    "Incomplete observation data: missing LIDAR data",
                    throttle_duration_sec=5.0
                )

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