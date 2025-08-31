#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Twist, Pose, Point, Quaternion
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TestNavigationNode(Node):
    def __init__(self):
        super().__init__('test_navigation_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        
        self.fire_position_pub = self.create_publisher(PointStamped, '/fire_position', qos)
        self.lidar_pub = self.create_publisher(LaserScan, '/world/forest_world/model/turtlebot4/link/lidar_link/sensor/lidar/scan', qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos)
        self.reward_sub = self.create_subscription(Float32, '/reward', self.reward_callback, qos)
        self.done_sub = self.create_subscription(Bool, '/done_flag', self.done_callback, qos)

        self.test_cases = [
            self.test_case_no_obstacles,
            self.test_case_static_obstacles,
            self.test_case_fire_loss,
            self.test_case_dynamic_obstacles
        ]
        self.current_test = 0
        self.test_start_time = None
        self.test_timeout = 60.0  # 60 seconds per test
        self.robot_position = [0.0, 0.0]
        self.robot_yaw = 0.0
        self.fire_position = [5.0, 0.0]
        self.obstacles = []
        self.cmd_vel = None
        self.total_reward = 0.0
        self.done = False

        self.timer = self.create_timer(0.1, self.run_test)
        self.get_logger().info("Test Navigation Node initialized")

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def reward_callback(self, msg):
        self.total_reward += msg.data

    def done_callback(self, msg):
        self.done = msg.data
        if self.done:
            self.get_logger().info(f"Test {self.current_test + 1} completed with total reward: {self.total_reward:.2f}")
            self.next_test()

    def run_test(self):
        if self.current_test >= len(self.test_cases):
            self.get_logger().info("All tests completed")
            self.destroy_node()
            rclpy.shutdown()
            return

        if self.test_start_time is None:
            self.test_start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f"Starting test case {self.current_test + 1}: {self.test_cases[self.current_test].__name__}")

        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.test_start_time > self.test_timeout:
            self.get_logger().warning(f"Test {self.current_test + 1} timed out")
            self.next_test()
            return

        self.test_cases[self.current_test]()
        self.publish_simulated_data()

    def next_test(self):
        self.current_test += 1
        self.test_start_time = None
        self.robot_position = [0.0, 0.0]
        self.robot_yaw = 0.0
        self.fire_position = [5.0, 0.0]
        self.obstacles = []
        self.total_reward = 0.0
        self.done = False
        if self.current_test < len(self.test_cases):
            self.get_logger().info(f"Preparing test case {self.current_test + 1}")
        else:
            self.get_logger().info("All tests completed")

    def test_case_no_obstacles(self):
        self.obstacles = []
        self.update_robot_position()

    def test_case_static_obstacles(self):
        self.obstacles = [(2.0, 1.0, 0.5), (2.0, -1.0, 0.5)]  # (x, y, radius)
        self.update_robot_position()

    def test_case_fire_loss(self):
        self.obstacles = []
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.test_start_time > 10.0:
            self.fire_position = None  # Simulate fire loss
        self.update_robot_position()

    def test_case_dynamic_obstacles(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        t = current_time - self.test_start_time
        self.obstacles = [(2.0 + math.sin(t * 0.5), 1.0, 0.5)]  # Moving obstacle
        self.update_robot_position()

    def update_robot_position(self):
        if self.cmd_vel is not None:
            dt = 0.1
            self.robot_position[0] += self.cmd_vel.linear.x * math.cos(self.robot_yaw) * dt
            self.robot_position[1] += self.cmd_vel.linear.x * math.sin(self.robot_yaw) * dt
            self.robot_yaw += self.cmd_vel.angular.z * dt
            self.robot_yaw = self.normalize_angle(self.robot_yaw)

    def normalize_angle(self, ang):
        return (ang + math.pi) % (2 * math.pi) - math.pi

    def publish_simulated_data(self):
        # Publish fire position
        if self.fire_position is not None:
            fire_msg = PointStamped()
            fire_msg.header.stamp = self.get_clock().now().to_msg()
            fire_msg.header.frame_id = 'world'
            fire_msg.point.x = self.fire_position[0]
            fire_msg.point.y = self.fire_position[1]
            fire_msg.point.z = 0.0
            self.fire_position_pub.publish(fire_msg)

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'world'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose = Pose()
        odom_msg.pose.pose.position = Point(x=self.robot_position[0], y=self.robot_position[1], z=0.0)
        q = self.yaw_to_quaternion(self.robot_yaw)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.odom_pub.publish(odom_msg)

        # Publish LIDAR
        lidar_msg = LaserScan()
        lidar_msg.header.stamp = self.get_clock().now().to_msg()
        lidar_msg.header.frame_id = 'lidar_link'
        lidar_msg.angle_min = -math.pi
        lidar_msg.angle_max = math.pi
        lidar_msg.angle_increment = math.pi / 180
        lidar_msg.range_min = 0.1
        lidar_msg.range_max = 10.0
        n = int((lidar_msg.angle_max - lidar_msg.angle_min) / lidar_msg.angle_increment)
        ranges = np.full(n, 10.0, dtype=np.float32)
        
        for obs in self.obstacles:
            ox, oy, radius = obs
            for i in range(n):
                angle = lidar_msg.angle_min + i * lidar_msg.angle_increment
                dx = ox - self.robot_position[0]
                dy = oy - self.robot_position[1]
                dist = math.sqrt(dx**2 + dy**2)
                theta = self.normalize_angle(math.atan2(dy, dx) - self.robot_yaw)
                if abs(theta - angle) < lidar_msg.angle_increment:
                    ranges[i] = max(lidar_msg.range_min, min(dist - radius, lidar_msg.range_max))
        
        lidar_msg.ranges = ranges.tolist()
        self.lidar_pub.publish(lidar_msg)

    def yaw_to_quaternion(self, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return [0.0, 0.0, sy, cy]

def main(args=None):
    rclpy.init(args=args)
    node = TestNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Test Navigation Node shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()