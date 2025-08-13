#!/usr/bin/env python3
import rclpy
import math
import numpy as np
import re
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PointStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

# Constants
MODEL_PREFIX = "fire_oka"
GROUND_PRIORITY_Z_THRESHOLD = 2.5

class FireSuppressionHandler(Node):
    # Track suppression mode from RL agent
    def __init__(self):
        super().__init__('fire_suppression_handler')
        self.suppression_mode = False
        self.create_subscription(Bool, '/suppression_mode', self.suppression_mode_callback, 10)

    def suppression_mode_callback(self, msg: Bool):
        self.suppression_mode = msg.data

        # Parameters
        self.declare_parameter('fov_deg', 30.0)
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('world_name', 'forest_world')
        self.declare_parameter('debug_detection', True)
        
        self.fov_deg = self.get_parameter('fov_deg').value
        self.max_range = self.get_parameter('max_range').value
        self.world_name = self.get_parameter('world_name').value
        self.debug_detection = self.get_parameter('debug_detection').value

        # State
        self.odom = None
        self.fire_models = {}  # model_name: (x, y, z)
        self.suppressed_fires = []  # List of (x, y) for suppressed fires
        self.suppressed_count = 0  # Track number of suppressed fires
        self.last_suppression_time = self.get_clock().now()
        self.last_tf_log_time = self.get_clock().now()
        self.tf_count = 0

        # Subscriptions
        self.create_subscription(Bool, '/trigger_vla', self.trigger_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(
            TFMessage, 
            f'/world/{self.world_name}/pose/info', 
            self.pose_info_cb, 
            10
        )

        # Publishers
        self.all_done_pub = self.create_publisher(Bool, '/all_fires_suppressed', 10)
        self.suppressed_count_pub = self.create_publisher(Int32, '/suppressed_fire_count', 10)
        self.suppressed_fire_pub = self.create_publisher(PointStamped, '/suppressed_fire_position', 10)

        self.get_logger().info("🚀 Fire Suppression Handler Node initialized")

    def pose_info_cb(self, msg: TFMessage):
        current_time = self.get_clock().now()
        self.tf_count += 1
        
        if self.debug_detection and (current_time - self.last_tf_log_time).nanoseconds > 5.0 * 1e9:
            self.get_logger().info(f"Received {self.tf_count} TF messages since last log")
            self.get_logger().info(f"Current TFMessage has {len(msg.transforms)} transforms")
            fire_frames = 0
            for transform in msg.transforms:
                frame_id = transform.child_frame_id
                if "fire" in frame_id.lower() or "light" in frame_id.lower():
                    fire_frames += 1
            self.get_logger().info(f"Found {fire_frames} fire-related frames in this message")
            count = 0
            for transform in msg.transforms:
                frame_id = transform.child_frame_id
                if "fire" in frame_id.lower() or "light" in frame_id.lower():
                    t = transform.transform.translation
                    self.get_logger().info(f"  Frame: {frame_id} at ({t.x:.2f}, {t.y:.2f}, {t.z:.2f})")
                    count += 1
                    if count >= 5:
                        break
            self.last_tf_log_time = current_time
            self.tf_count = 0

        for transform in msg.transforms:
            frame_id = transform.child_frame_id
            if "fire" not in frame_id.lower() and "light" not in frame_id.lower():
                continue
            model_name = self.resolve_model_name(frame_id)
            if not model_name:
                if self.debug_detection:
                    self.get_logger().info(f"Could not resolve model name from frame: {frame_id}")
                continue
            t = transform.transform.translation
            # Check if fire is already suppressed
            for sx, sy in self.suppressed_fires:
                if math.hypot(t.x - sx, t.y - sy) < 0.5:  # Threshold for matching
                    continue
            else:
                self.fire_models[model_name] = (t.x, t.y, t.z)
                if self.debug_detection:
                    self.get_logger().info(f"Tracking fire: {model_name} at ({t.x:.2f}, {t.y:.2f}, {t.z:.2f})")

    def resolve_model_name(self, frame_id):
        # Match fire_model_XX, fire_XX, or fire_XX_... patterns
        m = re.match(r'^(fire_model_\d+)', frame_id)
        if m:
            return m.group(1)
        m = re.match(r'^(fire_\d+)', frame_id)
        if m:
            return m.group(1)
        m = re.match(r'^(fire_\d+)_', frame_id)
        if m:
            return m.group(1)
        # Also check for double colon separated names
        if '::' in frame_id:
            parts = frame_id.split('::')
            for part in parts:
                m = re.match(r'^(fire_model_\d+)', part)
                if m:
                    return m.group(1)
                m = re.match(r'^(fire_\d+)', part)
                if m:
                    return m.group(1)
        # Fallback: search for fire_model_XX or fire_XX anywhere in the string
        match = re.search(r'(fire_model_\d+|fire_\d+)', frame_id)
        if match:
            return match.group(1)
        return None

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
        if not self.fire_models:
            self.log_current_fires()
            self.get_logger().warning("No fire models detected for suppression")
            return
        self.get_logger().info("Trigger received - locating fires")
        self.select_and_suppress_fire()
        self.last_suppression_time = current_time

    def log_current_fires(self):
        if self.fire_models:
            self.get_logger().info("Currently tracked fire models:")
            for name, pos in self.fire_models.items():
                self.get_logger().info(f"  {name}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
        else:
            self.get_logger().info("No fires in tracking list")

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def is_in_fov(self, model_pos, robot_pos, yaw):
        rx, ry, rz = robot_pos
        mx, my, mz = model_pos
        dx = mx - rx
        dy = my - ry
        dist = math.hypot(dx, dy)
        if dist > self.max_range:
            return False, dist, 0.0
        forward = np.array([math.cos(yaw), math.sin(yaw)])
        vec = np.array([dx, dy]) / (dist + 1e-9)
        dot = float(np.dot(forward, vec))
        return dot >= math.cos(math.radians(self.fov_deg)), dist, dot

    def select_and_suppress_fire(self):
        rx = self.odom.position.x
        ry = self.odom.position.y
        rz = self.odom.position.z
        yaw = self.quaternion_to_yaw(self.odom.orientation)
        candidates = []
        for model_name, model_pos in self.fire_models.items():
            x, y, z = model_pos
            in_fov, dist, dot = self.is_in_fov((x, y, z), (rx, ry, rz), yaw)
            ground_priority = -1.0 if z <= GROUND_PRIORITY_Z_THRESHOLD else 0.0
            candidates.append({
                'name': model_name,
                'dist': dist,
                'dot': dot,
                'in_fov': in_fov,
                'z': z,
                'score': dist + 0.5*(1.0 - dot) + ground_priority,
                'pos': (x, y)
            })
        if not candidates:
            self.get_logger().warning("No valid fire candidates found")
            return
        if self.debug_detection:
            self.get_logger().info("Fire candidates:")
            for c in candidates:
                self.get_logger().info(
                    f"  {c['name']}: Dist={c['dist']:.2f}m, Dot={c['dot']:.2f}, "
                    f"In FOV={c['in_fov']}, Z={c['z']:.2f}m, Score={c['score']:.2f}"
                )
        in_fov = [c for c in candidates if c['in_fov']]
        candidates_to_consider = in_fov if in_fov else candidates
        best_candidate = min(candidates_to_consider, key=lambda x: x['score'])
    # Proximity check: only suppress if within 1.2m AND suppression_mode is True
        if best_candidate['dist'] <= 1.2 and self.suppression_mode:
            self.get_logger().info(
                f"Selected fire: {best_candidate['name']} "
                f"Dist: {best_candidate['dist']:.2f}m "
                f"Dot: {best_candidate['dot']:.2f} "
                f"In FOV: {best_candidate['in_fov']} "
                f"Z: {best_candidate['z']:.2f}m"
            )
            # Mark fire as suppressed
            self.suppressed_fires.append(best_candidate['pos'])
            self.suppressed_count += 1
            # Publish suppressed fire position
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = 'map'
            point_msg.point.x = float(best_candidate['pos'][0])
            point_msg.point.y = float(best_candidate['pos'][1])
            point_msg.point.z = 0.0
            self.suppressed_fire_pub.publish(point_msg)
            # Publish count
            self.suppressed_count_pub.publish(Int32(data=self.suppressed_count))
            # Remove from fire_models to prevent re-suppression
            if best_candidate['name'] in self.fire_models:
                del self.fire_models[best_candidate['name']]
                self.get_logger().info(f"Marked {best_candidate['name']} as suppressed")
            if not self.fire_models:
                self.all_done_pub.publish(Bool(data=True))
                self.get_logger().info(f"🏆 All fires suppressed! Total: {self.suppressed_count}")
        else:
            self.get_logger().info(
                f"Fire {best_candidate['name']} is too far to suppress (Dist: {best_candidate['dist']:.2f}m)"
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