#!/usr/bin/env python3
import rclpy
import math
import numpy as np
import re
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.srv import DeleteEntity
from ros_gz_interfaces.msg import Entity
from rclpy.executors import MultiThreadedExecutor

# Constants
MODEL_PREFIX = "fire_model_"
GROUND_PRIORITY_Z_THRESHOLD = 2.5
DELETE_TIMEOUT = 10.0  # Increased timeout

class FireSuppressionIgnitionNode(Node):
    def __init__(self):
        super().__init__('fire_suppression_ignition_node')

        # Parameters
        self.declare_parameter('fov_deg', 30.0)
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('world_name', 'forest_world')
        
        self.fov_deg = self.get_parameter('fov_deg').value
        self.max_range = self.get_parameter('max_range').value
        self.world_name = self.get_parameter('world_name').value
        
        # Service endpoint
        self.delete_service = f'/world/{self.world_name}/remove'

        # State
        self.odom = None
        self.fire_models = {}  # model_name: (x, y, z)
        self.last_deletion_time = self.get_clock().now().to_msg()

        # Subscriptions
        self.create_subscription(Bool, '/trigger_vla', self.trigger_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(
            TFMessage, 
            f'/world/{self.world_name}/pose/info', 
            self.pose_info_cb, 
            10
        )

        # Service client
        self.delete_client = self.create_client(DeleteEntity, self.delete_service)
        self.get_logger().info(f"Waiting for delete service: {self.delete_service}")
        if not self.delete_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f"Delete service not available: {self.delete_service}")
        else:
            self.get_logger().info(f"Connected to delete service: {self.delete_service}")

    # ---------------- callbacks ----------------
    def odom_cb(self, msg: Odometry):
        self.odom = msg.pose.pose

    def pose_info_cb(self, msg: TFMessage):
        """Store positions of fire models from TF messages"""
        for transform in msg.transforms:
            frame_id = transform.child_frame_id
            
            # Only consider fire-related frames
            if MODEL_PREFIX not in frame_id:
                continue
                
            # Extract model name using hierarchical resolution
            model_name = self.resolve_model_name(frame_id)
            if not model_name:
                continue
                
            t = transform.transform.translation
            self.fire_models[model_name] = (t.x, t.y, t.z)
            
            self.get_logger().debug(f"Detected frame: {frame_id} → Model: {model_name}")

    def resolve_model_name(self, frame_id):
        """
        Resolve root model name from various frame patterns
        """
        # Case 1: Direct model name
        if re.match(fr"^{MODEL_PREFIX}\d+$", frame_id):
            return frame_id
            
        # Case 2: Scoped names (model::link)
        if '::' in frame_id:
            parts = frame_id.split('::')
            if re.match(fr"^{MODEL_PREFIX}\d+$", parts[0]):
                return parts[0]
        
        # Case 3: Component names (with prefixes)
        pattern = fr"({MODEL_PREFIX}\d+)(?:_(fire_main|fire_60|fire_120|light)_\d+)?"
        match = re.match(pattern, frame_id)
        if match:
            return match.group(1)
            
        return None

    def trigger_cb(self, msg: Bool):
        if not msg.data:
            return
            
        # Prevent rapid triggering
        current_time = self.get_clock().now().to_msg()
        if (current_time.sec - self.last_deletion_time.sec) < 3:
            self.get_logger().warn("Trigger ignored - too frequent")
            return
            
        if self.odom is None:
            self.get_logger().warning("Trigger received but missing odometry data")
            return
            
        if not self.fire_models:
            self.get_logger().warning("No fire models detected")
            return
            
        self.get_logger().info("Trigger received - locating fires")
        self.select_and_remove_fire()
        self.last_deletion_time = current_time

    # ---------------- helpers ----------------
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def is_in_fov(self, model_pos, robot_pos, yaw):
        """Check if model is within field of view"""
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

    def select_and_remove_fire(self):
        """Main logic to select and remove nearest fire"""
        # Get robot state
        rx = self.odom.position.x
        ry = self.odom.position.y
        rz = self.odom.position.z
        yaw = self.quaternion_to_yaw(self.odom.orientation)
        
        # Evaluate candidates
        candidates = []
        for model_name, model_pos in self.fire_models.items():
            x, y, z = model_pos
            in_fov, dist, dot = self.is_in_fov((x, y, z), (rx, ry, rz), yaw)
            
            # Ground priority bonus
            ground_priority = -1.0 if z <= GROUND_PRIORITY_Z_THRESHOLD else 0.0
            
            candidates.append({
                'name': model_name,
                'dist': dist,
                'dot': dot,
                'in_fov': in_fov,
                'z': z,
                'score': dist + 0.5*(1.0 - dot) + ground_priority
            })
        
        if not candidates:
            self.get_logger().warning("No valid fire candidates found")
            return
            
        # Select best candidate (prioritize in FOV)
        in_fov = [c for c in candidates if c['in_fov']]
        candidates_to_consider = in_fov if in_fov else candidates
        best_candidate = min(candidates_to_consider, key=lambda x: x['score'])
        
        self.get_logger().info(
            f"Selected fire: {best_candidate['name']} "
            f"Dist: {best_candidate['dist']:.2f}m "
            f"Dot: {best_candidate['dot']:.2f} "
            f"In FOV: {best_candidate['in_fov']} "
            f"Z: {best_candidate['z']:.2f}m"
        )
        
        # Delete selected fire by name
        success = self.delete_fire(best_candidate['name'])
        
        # Only remove from tracking if deletion was successful
        if success and best_candidate['name'] in self.fire_models:
            del self.fire_models[best_candidate['name']]

    def delete_fire(self, model_name):
        """Delete entity from simulation by name"""
        req = DeleteEntity.Request()
        req.entity = Entity(name=model_name, type=Entity.MODEL)
        
        try:
            self.get_logger().info(f"Requesting deletion: {model_name}")
            future = self.delete_client.call_async(req)
            start_time = self.get_clock().now()
            
            # Wait for response with timeout
            while (self.get_clock().now() - start_time).nanoseconds < DELETE_TIMEOUT * 1e9:
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    break
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(f"✅ Successfully deleted {model_name}")
                    return True
                else:
                    self.get_logger().warn(f"Service reported failure deleting {model_name}")
                    return False
            else:
                self.get_logger().warn(f"Delete request timed out for {model_name}")
                future.cancel()
                return False
        except Exception as e:
            self.get_logger().error(f"Exception during delete: {str(e)}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = FireSuppressionIgnitionNode()
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