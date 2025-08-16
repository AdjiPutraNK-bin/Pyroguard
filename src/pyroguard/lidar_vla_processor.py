#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
import numpy as np
import math

class LidarVlaProcessorNode(Node):
    def __init__(self):
        super().__init__('lidar_vla_processor_node')
        
        # Parameters
        self.declare_parameter('min_safe_distance', 0.3)
        self.declare_parameter('suppression_distance', 2.0)
        self.declare_parameter('lidar_topic', '/world/forest_world/model/turtlebot4/link/lidar_link/sensor/lidar/scan')
        self.declare_parameter('world_name', 'forest_world')
        self.declare_parameter('camera_fov_deg', 60.0)  # Camera horizontal FOV in degrees
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.suppression_distance = self.get_parameter('suppression_distance').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.world_name = self.get_parameter('world_name').value
        self.camera_fov_deg = self.get_parameter('camera_fov_deg').value
        self.camera_fov_rad = math.radians(self.camera_fov_deg)

        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, self.lidar_topic, self.lidar_callback, 10)
        self.vla_sub = self.create_subscription(
            Float32MultiArray, '/vla_output', self.vla_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.obs_pub = self.create_publisher(Float32MultiArray, '/obs', 10)
        self.fire_position_pub = self.create_publisher(PointStamped, '/fire_position', 10)

        # State
        self.vla_data = None  # [fire_or_no, fire_size, -1, bbox_x]
        self.lidar_min_distance = float('inf')
        self.robot_pose = None
        self.lidar_ranges = None
        self.lidar_angle_min = None
        self.lidar_angle_increment = None

        # Timer for publishing observations
        self.timer = self.create_timer(0.1, self.publish_observation)
        
        self.get_logger().info("🚀 LIDAR-VLA Processor Node initialized")

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array(msg.ranges)
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_increment = msg.angle_increment
        finite_ranges = self.lidar_ranges[np.isfinite(self.lidar_ranges)]
        self.lidar_min_distance = float(np.min(finite_ranges)) if len(finite_ranges) > 0 else float('inf')

    def vla_callback(self, msg):
        if len(msg.data) >= 4:
            self.vla_data = np.array(msg.data[:4], dtype=np.float32)
        else:
            self.get_logger().error(f"Invalid VLA data: expected at least 4 elements, got {len(msg.data)}")
            self.vla_data = None

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
        if not hasattr(self, '_robot_frame_logged'):
            self.get_logger().info(f"Robot odometry frame: {msg.header.frame_id}")
            self._robot_frame_logged = True

    def compute_angle_to_fire(self):
        if self.vla_data is None or len(self.vla_data) < 4:
            return 0.0
        bbox_x = self.vla_data[3]
        normalized_x = bbox_x - 0.5
        angle_to_fire = normalized_x * (self.camera_fov_rad / 2)
        return angle_to_fire

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_observation(self):
        vla_available = self.vla_data is not None
        lidar_available = self.lidar_min_distance != float('inf')

        if vla_available and lidar_available:
            fire_or_no = self.vla_data[0]
            fire_size = self.vla_data[1]
            angle_to_fire = self.compute_angle_to_fire() if fire_or_no > 0.5 else 0.0

            fire_distance = float('inf')
            if fire_or_no > 0.5:
                angle = angle_to_fire
                if self.lidar_angle_increment != 0 and self.lidar_ranges is not None:
                    index = int((angle - self.lidar_angle_min) / self.lidar_angle_increment)
                    if 0 <= index < len(self.lidar_ranges) and np.isfinite(self.lidar_ranges[index]):
                        fire_distance = self.lidar_ranges[index]

            # Publish observation
            obs = np.array([fire_or_no, fire_size, self.lidar_min_distance, angle_to_fire, fire_distance], dtype=np.float32)
            msg = Float32MultiArray()
            msg.data = obs.tolist()
            self.obs_pub.publish(msg)

            # Publish fire position if detected
            if fire_or_no > 0.5 and fire_distance != float('inf') and self.robot_pose is not None:
                rx, ry = self.robot_pose.position.x, self.robot_pose.position.y
                yaw = self.quaternion_to_yaw(self.robot_pose.orientation)
                fx = rx + fire_distance * math.cos(yaw + angle_to_fire)
                fy = ry + fire_distance * math.sin(yaw + angle_to_fire)
                fz = 1.0  # Assume ground height
                point_msg = PointStamped()
                point_msg.header.stamp = self.get_clock().now().to_msg()
                point_msg.header.frame_id = 'map'
                point_msg.point.x = fx
                point_msg.point.y = fy
                point_msg.point.z = fz
                self.fire_position_pub.publish(point_msg)
                self.get_logger().info(f"Published fire position: ({fx:.2f}, {fy:.2f}, {fz:.2f})")

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