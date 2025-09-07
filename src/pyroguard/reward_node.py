#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import numpy as np
import math

class RewardNode(Node):
    def __init__(self):
        super().__init__('reward_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_safe_distance', 0.8),
                ('suppression_distance', 4.0),
                ('fire_reward', 100.0),
                ('distance_reward_scale', 2.0),
                ('alignment_penalty_scale', 0.5),
                ('obstacle_penalty_scale', 1.0),
                ('stuck_penalty', -0.5),
                ('lidar_topic', '/world/forest_world/model/turtlebot4/link/lidar_link/sensor/lidar/scan'),
            ]
        )

        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.suppression_distance = self.get_parameter('suppression_distance').value
        self.fire_reward = self.get_parameter('fire_reward').value
        self.distance_reward_scale = self.get_parameter('distance_reward_scale').value
        self.alignment_penalty_scale = self.get_parameter('alignment_penalty_scale').value
        self.obstacle_penalty_scale = self.get_parameter('obstacle_penalty_scale').value
        self.stuck_penalty = self.get_parameter('stuck_penalty').value
        self.lidar_topic = self.get_parameter('lidar_topic').value

        self.reward_pub = self.create_publisher(Float32, '/reward', 10)
        self.done_pub = self.create_publisher(Bool, '/done_flag', 10)
        self.lidar_sub = self.create_subscription(LaserScan, self.lidar_topic, self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.fire_position_sub = self.create_subscription(PointStamped, '/fire_position', self.fire_position_callback, 10)
        self.suppression_mode_sub = self.create_subscription(Bool, '/suppression_mode', self.suppression_mode_callback, 10)

        self.lidar_ranges = None
        self.robot_pose = None
        self.fire_position = None
        self.suppression_mode = False
        self.previous_fire_distance = None
        self.previous_action = None
        self.action_counts = {}
        self.stuck_counter = 0

        self.timer = self.create_timer(0.1, self.compute_reward)
        self.get_logger().info("Reward Node initialized")

    def lidar_callback(self, msg):
        try:
            self.lidar_ranges = np.array(msg.ranges, dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f"LIDAR callback failed: {str(e)}")
            self.lidar_ranges = None

    def odom_callback(self, msg):
        try:
            self.robot_pose = msg.pose.pose
        except Exception as e:
            self.get_logger().error(f"Odometry callback failed: {str(e)}")
            self.robot_pose = None

    def fire_position_callback(self, msg):
        try:
            self.fire_position = (msg.point.x, msg.point.y)
        except Exception:
            self.fire_position = None

    def suppression_mode_callback(self, msg):
        self.suppression_mode = msg.data

    def compute_reward(self):
        reward = 0.0
        done = False

        if self.lidar_ranges is None or self.robot_pose is None:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - getattr(self, '_last_missing_reward_data_warn', 0) >= 10.0:
                self.get_logger().warning("Missing LIDAR or odometry data, publishing zero reward")
                self._last_missing_reward_data_warn = current_time
            self.reward_pub.publish(Float32(data=0.0))
            self.done_pub.publish(Bool(data=False))
            return

        # Obstacle penalty
        ranges = np.where(np.isfinite(self.lidar_ranges), self.lidar_ranges, 10.0)
        ranges = np.where(ranges <= 0.0, ranges, 10.0)
        min_distance = float(np.min(ranges))
        if min_distance < self.min_safe_distance:
            reward -= self.obstacle_penalty_scale * (self.min_safe_distance - min_distance)

        # Fire proximity and alignment reward
        if self.fire_position is not None and self.robot_pose is not None:
            dx = self.fire_position[0] - self.robot_pose.position.x
            dy = self.fire_position[1] - self.robot_pose.position.y
            fire_distance = math.sqrt(dx**2 + dy**2)
            
            # Distance reward
            if self.previous_fire_distance is not None:
                distance_diff = self.previous_fire_distance - fire_distance
                reward += self.distance_reward_scale * distance_diff
            self.previous_fire_distance = fire_distance

            # Alignment penalty
            yaw = self.quaternion_to_yaw(self.robot_pose.orientation)
            
            # Validate inputs to prevent RuntimeWarning
            if not (math.isfinite(dx) and math.isfinite(dy) and math.isfinite(yaw)):
                self.get_logger().debug(f"Invalid alignment data: dx={dx}, dy={dy}, yaw={yaw}")
            else:
                fire_angle = math.atan2(dy, dx) - yaw
                fire_angle = self.normalize_angle(fire_angle)
                reward -= self.alignment_penalty_scale * abs(fire_angle)

            # Suppression reward
            if fire_distance <= self.suppression_distance and self.suppression_mode:
                reward += self.fire_reward
                done = True
                self.get_logger().info(f"Fire suppressed at distance {fire_distance:.2f}, rewarding {self.fire_reward}")
        else:
            self.previous_fire_distance = None

        # Publish reward and done flag
        self.reward_pub.publish(Float32(data=float(reward)))
        self.done_pub.publish(Bool(data=done))
        self.get_logger().debug(f"Reward: {reward:.2f}, Done: {done}")

    def quaternion_to_yaw(self, q):
        try:
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)
        except Exception:
            self.get_logger().error("Failed to compute yaw from quaternion")
            return 0.0

    def normalize_angle(self, ang):
        return (ang + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    node = RewardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Reward Node shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()