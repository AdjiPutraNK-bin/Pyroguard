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
        self.vla_data = None  # [fire_or_no, fire_size, fire_id, bbox_x]
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
        if len(msg.data) == 3:
            # Legacy: [fire_or_no, fire_size, fire_id]
            self.vla_data = np.array(list(msg.data) + [0.5, 0.0, 0.0, 0.0], dtype=np.float32)  # Default bbox_x=0.5, pose=0
        elif len(msg.data) == 4:
            # [fire_or_no, fire_size, fire_id, bbox_x]
            self.vla_data = np.array(list(msg.data) + [0.0, 0.0, 0.0], dtype=np.float32)
        elif len(msg.data) == 7:
            # [fire_or_no, fire_size, fire_id, bbox_x, fire_x, fire_y, fire_z]
            self.vla_data = np.array(msg.data, dtype=np.float32)
        else:
            self.get_logger().error(f"Invalid VLA data: expected 3, 4, or 7 elements, got {len(msg.data)}")

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
                        if math.hypot(t.x - sx, t.y - sy) < 1.2:  # Threshold for matching
                            break
                    else:
                        self.fire_poses[model_name] = (t.x, t.y, t.z)

    def resolve_model_name(self, frame_id):
        MODEL_PREFIX = "fire_"
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
        if not self.fire_poses or self.robot_pose is None or self.vla_data is None or len(self.vla_data) < 3:
            self.get_logger().warning("Missing robot/fire poses or fire ID, using angle_to_fire=0.0", throttle_duration_sec=5.0)
            return 0.0
        rx, ry = self.robot_pose.position.x, self.robot_pose.position.y
        yaw = self.quaternion_to_yaw(self.robot_pose.orientation)
        fire_id_num = int(self.vla_data[2])
        target_angle = 0.0
        # Find fire pose by ID
        fire_key = f"fire_{fire_id_num}"
        if fire_key in self.fire_poses:
            fx, fy = self.fire_poses[fire_key][0], self.fire_poses[fire_key][1]
            angle = math.atan2(fy - ry, fx - rx) - yaw
            target_angle = np.arctan2(np.sin(angle), np.cos(angle))
            # Only log when angle changes sign (crosses zero)
            if not hasattr(self, 'last_angle_sign'):
                self.last_angle_sign = np.sign(target_angle)
            if np.sign(target_angle) != self.last_angle_sign:
                self.get_logger().info(f"Angle sign change: robot=({rx:.2f},{ry:.2f}), yaw={yaw:.2f}, fire=({fx:.2f},{fy:.2f}), raw_angle={angle:.2f}, target_angle={target_angle:.2f}")
            self.last_angle_sign = np.sign(target_angle)
        else:
            self.get_logger().warning(f"Fire ID {fire_key} not found in fire_poses", throttle_duration_sec=5.0)
        return target_angle

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_observation(self):
        if self.vla_data is not None and self.lidar_min_distance != float('inf'):
            # Only provide angle_to_fire if fire is detected
            if self.vla_data[0] > 0.5:
                angle_to_fire = self.compute_angle_to_fire()
            else:
                angle_to_fire = 0.0
            bbox_x = self.vla_data[3] if len(self.vla_data) > 3 else 0.5
            fire_pose = self.vla_data[4:7] if len(self.vla_data) >= 7 else [0.0, 0.0, 0.0]
            # Log when bbox_x crosses center (0.5)
            if not hasattr(self, 'last_bbox_sign'):
                self.last_bbox_sign = np.sign(bbox_x - 0.5)
            if np.sign(bbox_x - 0.5) != self.last_bbox_sign:
                self.get_logger().info(f"bbox_x center crossing: bbox_x={bbox_x:.2f}")
            self.last_bbox_sign = np.sign(bbox_x - 0.5)
            # Publish obs: [fire_or_no, fire_size, lidar_min_distance, angle_to_fire, bbox_x, fire_x, fire_y, fire_z]
            obs = np.concatenate([self.vla_data[:2], [self.lidar_min_distance, angle_to_fire, bbox_x], fire_pose])
            msg = Float32MultiArray()
            msg.data = obs.tolist()
            self.obs_pub.publish(msg)
            self.get_logger().info("Received both VLA and LIDAR data, publishing observation.")
        else:
            if self.vla_data is None and self.lidar_min_distance == float('inf'):
                self.get_logger().debug(
                    "Incomplete observation data: missing both VLA and LIDAR"
                )
            elif self.vla_data is None:
                self.get_logger().debug(
                    "Incomplete observation data: missing VLA data"
                )
            elif self.lidar_min_distance == float('inf'):
                self.get_logger().debug(
                    "Incomplete observation data: missing LIDAR data"
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