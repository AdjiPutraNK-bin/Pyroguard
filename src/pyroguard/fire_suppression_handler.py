#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

class FireSuppressionHandler(Node):
    def __init__(self):
        super().__init__('fire_suppression_handler')
        self.suppression_mode = False
        # Parameters
        self.declare_parameter('fov_deg', 60.0)
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('world_name', 'forest_world')
        self.declare_parameter('debug_detection', True)
        self.declare_parameter('suppression_distance', 2.0)  # Universal suppression distance
        self.declare_parameter('num_fires', 8)  # Total number of fires
        self.fov_deg = self.get_parameter('fov_deg').value
        self.max_range = self.get_parameter('max_range').value
        self.world_name = self.get_parameter('world_name').value
        self.debug_detection = self.get_parameter('debug_detection').value
        self.suppression_distance = self.get_parameter('suppression_distance').value
        self.num_fires = self.get_parameter('num_fires').value

        # State
        self.odom = None
        self.suppressed_fires = []  # List of (x, y) for suppressed fires
        self.suppressed_count = 0  # Track number of suppressed fires
        self.last_suppression_time = self.get_clock().now()
        self.current_fire_position = None

        # Subscriptions
        self.create_subscription(Bool, '/suppression_mode', self.suppression_mode_callback, 10)
        self.create_subscription(Bool, '/trigger_vla', self.trigger_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PointStamped, '/fire_position', self.fire_position_callback, 10)

        # Publishers
        self.all_done_pub = self.create_publisher(Bool, '/all_fires_suppressed', 10)
        self.suppressed_count_pub = self.create_publisher(Int32, '/suppressed_fire_count', 10)
        self.suppressed_fire_pub = self.create_publisher(PointStamped, '/suppressed_fire_position', 10)

        self.get_logger().info("🚀 Fire Suppression Handler Node initialized")

    def suppression_mode_callback(self, msg: Bool):
        self.suppression_mode = msg.data

    def fire_position_callback(self, msg: PointStamped):
        self.current_fire_position = (msg.point.x, msg.point.y)
        self.get_logger().info(f"Updated current fire position: ({msg.point.x:.2f}, {msg.point.y:.2f})")

    def trigger_cb(self, msg: Bool):
        if not msg.data:
            return
        current_time = self.get_clock().now()
        if (current_time - self.last_suppression_time).nanoseconds < 3e9:
            self.get_logger().warn("Trigger ignored - too frequent")
            return
        if self.odom is None:
            self.get_logger().warning("Trigger received but missing odometry data")
            return
        if self.current_fire_position is None:
            self.get_logger().warning("No current fire position available for suppression")
            return
        self.get_logger().info("Trigger received - attempting suppression")
        self.select_and_suppress_fire()
        self.last_suppression_time = current_time

    def select_and_suppress_fire(self):
        rx = self.odom.position.x
        ry = self.odom.position.y
        dist = math.hypot(self.current_fire_position[0] - rx, self.current_fire_position[1] - ry)
        if dist <= self.suppression_distance:
            self.get_logger().info(
                f"Suppressing fire at dist {dist:.2f}m"
            )
            self.suppressed_fires.append(self.current_fire_position)
            self.suppressed_count += 1
            # Publish suppressed fire position
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = 'map'
            point_msg.point.x = self.current_fire_position[0]
            point_msg.point.y = self.current_fire_position[1]
            point_msg.point.z = 0.0
            self.suppressed_fire_pub.publish(point_msg)
            # Publish count
            self.suppressed_count_pub.publish(Int32(data=self.suppressed_count))
            self.current_fire_position = None  # Reset after suppression
            if self.suppressed_count >= self.num_fires:
                self.all_done_pub.publish(Bool(data=True))
                self.get_logger().info(f"🏆 All fires suppressed! Total: {self.suppressed_count}")
        else:
            self.get_logger().info(
                f"Fire too far to suppress: {dist:.2f}m > {self.suppression_distance}m"
            )

    def odom_cb(self, msg: Odometry):
        self.odom = msg.pose.pose
        if self.debug_detection:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.get_logger().info(f"Updated odometry: ({x:.2f}, {y:.2f})", throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = FireSuppressionHandler()
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