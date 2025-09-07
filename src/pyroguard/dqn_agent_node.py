#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import torch
import numpy as np
import os
from std_msgs.msg import Float32MultiArray, Float32, Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped
from .dqn_agent import DQNAgent
from collections import deque
import math

class DQNAgentNode(Node):
    def __init__(self):
        super().__init__('dqn_agent_node')
        self.suppressed_fire_positions = []
        self.detected_fire_positions = []
        self.suppression_mode_pub = self.create_publisher(Bool, '/suppression_mode', 10)
        self.vla_pub = self.create_publisher(Bool, '/trigger_vla', 10)

        self.fire_position_sub = self.create_subscription(PointStamped, '/fire_position', self.fire_position_callback, 10)
        self.suppressed_fire_sub = self.create_subscription(PointStamped, '/suppressed_fire_position', self.suppressed_fire_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Add service for getting fire detection info
        self.fire_info_service = self.create_service(Trigger, 'get_fire_detection_info', self.handle_fire_info_request)
        
        # Add service for resetting mission
        self.reset_mission_service = self.create_service(Trigger, 'reset_mission', self.handle_reset_mission_request)

        self.fire_detected_buffer = deque([False]*5, maxlen=5)
        self.avoid_kp = 0.6
        self.avoid_kd = 0.2
        self.avoid_last_error = 0.0

        self.declare_parameters(
            namespace='',
            parameters=[
                ('obs_size', 5),
                ('action_size', 6),
                ('model_path', 'dqn_model.pth'),
                ('epsilon_start', 0.9),
                ('epsilon_end', 0.05),
                ('epsilon_decay', 1000),
                ('save_interval', 1000),
                ('min_safe_distance', 0.8),
                ('suppression_distance', 4.0),
                ('action_repeat_penalty', -0.5),
                ('mode', 'train_online'),
                ('lidar_topic', '/world/forest_world/model/turtlebot4/link/lidar_link/sensor/lidar/scan'),
                ('angle_threshold', 0.1),
                ('turn_speed', 0.6),
                ('forward_speed', 0.3),
                ('slow_forward_speed', 0.15),
                ('backward_speed', -0.15),
                ('avoidance_arc_margin', 0.8),
                ('avoidance_max_angular', 0.5),
                ('avoid_front_angle_deg', 90.0),
                ('target_lock_threshold', 0.6),
                ('min_suppression_time', 5.0),
                ('approach_interrupt_timeout', 1.2),
                ('emergency_escape_duration', 0.8),
                ('emergency_back_speed', -0.12),
                ('emergency_threshold', 0.10),
                ('approach_escape_scale', 0.5),
                ('approach_avoid_linear', 0.08),
                ('approach_avoid_angular', 0.2),
                ('approach_correction_gain', 0.6),
                ('front_percentile', 50),
                ('min_obstacle_beams', 5),
                ('avoidance_window_deg', 120.0),
                ('path_cost_distance_weight', 0.7),
                ('path_cost_obstacle_weight', 0.3),
                ('return_to_base_timeout', 15.0),
                ('avoidance_timeout', 2.0),
            ]
        )

        self.obs_size = self.get_parameter('obs_size').value
        self.action_size = self.get_parameter('action_size').value
        self.model_path = self.get_parameter('model_path').value
        self.epsilon_start = self.get_parameter('epsilon_start').value
        self.epsilon_end = self.get_parameter('epsilon_end').value
        self.epsilon_decay = self.get_parameter('epsilon_decay').value
        self.save_interval = self.get_parameter('save_interval').value
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.suppression_distance = self.get_parameter('suppression_distance').value
        self.action_repeat_penalty = self.get_parameter('action_repeat_penalty').value
        self.mode = self.get_parameter('mode').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.angle_threshold = self.get_parameter('angle_threshold').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.slow_forward_speed = self.get_parameter('slow_forward_speed').value
        self.backward_speed = self.get_parameter('backward_speed').value
        self.avoidance_arc_margin = self.get_parameter('avoidance_arc_margin').value
        self.avoidance_max_angular = self.get_parameter('avoidance_max_angular').value
        self.avoid_front_angle = math.radians(self.get_parameter('avoid_front_angle_deg').value)
        self.target_lock_threshold = self.get_parameter('target_lock_threshold').value
        self.min_suppression_time = self.get_parameter('min_suppression_time').value
        self.approach_interrupt_timeout = self.get_parameter('approach_interrupt_timeout').value
        self.emergency_escape_duration = self.get_parameter('emergency_escape_duration').value
        self.emergency_back_speed = self.get_parameter('emergency_back_speed').value
        self.emergency_threshold = self.get_parameter('emergency_threshold').value
        self.approach_escape_scale = self.get_parameter('approach_escape_scale').value
        self.approach_avoid_linear = self.get_parameter('approach_avoid_linear').value
        self.approach_avoid_angular = self.get_parameter('approach_avoid_angular').value
        self.approach_correction_gain = self.get_parameter('approach_correction_gain').value
        self.front_percentile = self.get_parameter('front_percentile').value
        self.min_obstacle_beams = int(self.get_parameter('min_obstacle_beams').value)
        self.avoidance_window = math.radians(self.get_parameter('avoidance_window_deg').value)
        self.path_cost_distance_weight = self.get_parameter('path_cost_distance_weight').value
        self.path_cost_obstacle_weight = self.get_parameter('path_cost_obstacle_weight').value
        self.return_to_base_timeout = self.get_parameter('return_to_base_timeout').value
        self.avoidance_timeout = self.get_parameter('avoidance_timeout').value

        if self.mode not in ['collect', 'train_online', 'inference']:
            raise ValueError("Invalid mode: choose 'collect', 'train_online', or 'inference'")

        self.agent = DQNAgent(self.obs_size, self.action_size)
        
        if self.mode != 'collect':
            try:
                if os.path.exists(self.model_path):
                    checkpoint = torch.load(self.model_path, map_location=torch.device('cpu'))
                    self.agent.policy_net.load_state_dict(checkpoint['model_state_dict'])
                    self.agent.target_net.load_state_dict(checkpoint['target_state_dict'])
                    self.agent.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
                    self.get_logger().info("DQN model loaded successfully")
                else:
                    self.get_logger().warning("No model found - using random weights")
            except (FileNotFoundError, RuntimeError) as e:
                self.get_logger().error(f"Model loading failed: {str(e)} - continuing with random weights")

        self.obs_sub = self.create_subscription(Float32MultiArray, '/obs', self.obs_callback, 10)
        self.reward_sub = self.create_subscription(Float32, '/reward', self.reward_callback, 10)
        self.done_sub = self.create_subscription(Bool, '/done_flag', self.done_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, self.lidar_topic, self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_obs = None
        self.previous_obs = None
        self.reward = 0.0
        self.done = False
        self.step_count = 0
        self.episode_count = 0
        self.lidar_ranges = None
        self.lidar_angle_increment = None
        self.lidar_angle_min = None
        self.suppression_mode = False
        self.approach_mode = False
        self.locked_angle = 0.0
        self.approach_lost_since = None
        self.post_avoid_recenter = False
        self.escape_state = None
        self.escape_end_time = 0.0
        self.escape_rotation_side = 0.0
        self.avoidance_start_time = None
        self.avoidance_count = 0
        self.robot_pose = None

        self.recent_actions = deque(maxlen=10)
        self.action_counts = {i: 0 for i in range(self.action_size)}
        self.stuck_counter = 0

        self.current_target_fire = None
        self.target_lock_start_time = None
        self.current_fire_world = None

        # Fire suppression tracking
        self.suppressed_fire_positions = []
        self.fire_detected_buffer = deque(maxlen=5)

        self.action_map = {
            0: "move_forward",
            1: "turn_left",
            2: "turn_right",
            3: "stop",
            4: "move_backward",
            5: "slow_forward"
        }

        self.obs_received = False
        self.lidar_received = False
        self.odom_received = False
        self.is_ready = False
        self.readiness_timer = self.create_timer(1.0, self.check_readiness)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"🤖 DQN Agent Node initialized in mode: {self.mode}")

        self.recenter_checkpoints = []
        self.last_recenter_distance = None

        # Return-to-base functionality
        self.starting_position = None
        self.last_fire_detection_time = None
        self.return_to_base_mode = False
        self.return_to_base_timer = self.create_timer(1.0, self.check_return_to_base)
        self.no_fire_timeout = self.return_to_base_timeout  # Use parameter value
        
        # Debug variables for return-to-base timing
        self.return_to_base_start_time = None
        self.last_new_fire_detection_time = None
        self.debug_log_timer = self.create_timer(5.0, self.debug_return_to_base_status)
        
        # Search mode variables to prevent spinning
        self.search_mode = False
        self.search_start_time = None
        self.last_action_time = None
        self.spin_timeout = 10.0  # Stop spinning after 10 seconds
        
        # Mission complete flag
        self.mission_complete = False
        
        # Return-to-base navigation variables
        self.last_return_position = None
        self.stuck_counter = 0
        self.rotation_start_time = None

    def check_readiness(self):
        if self.current_obs is not None:
            self.obs_received = True
        if self.lidar_ranges is not None:
            self.lidar_received = True
        if self.robot_pose is not None:
            self.odom_received = True
        if self.obs_received and self.lidar_received and self.odom_received:
            self.is_ready = True
            self.get_logger().info("All required topics received, node ready")
            self.readiness_timer.cancel()
        else:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - getattr(self, '_last_topics_wait_warn', 0) >= 10.0:
                self.get_logger().warning("Waiting for required topics: obs=%s, lidar=%s, odom=%s" % 
                                         (self.obs_received, self.lidar_received, self.odom_received))
                self._last_topics_wait_warn = current_time

    def obs_callback(self, msg):
        if len(msg.data) == self.obs_size:
            self.current_obs = np.array(msg.data, dtype=np.float32)
            self.obs_received = True
        else:
            self.get_logger().error(f"Invalid observation size: expected {self.obs_size}, got {len(msg.data)}")
            self.current_obs = None

    def reward_callback(self, msg):
        self.reward = msg.data

    def done_callback(self, msg):
        self.done = msg.data
        if self.done:
            self.episode_count += 1
            self.get_logger().info(
                f"🏁 Episode {self.episode_count} ended. Steps: {self.step_count}, Final reward: {self.reward:.2f}"
            )
            self.previous_obs = None
            self.reward = 0.0
            self.done = False
            self.recent_actions.clear()
            self.action_counts = {i: 0 for i in range(self.action_size)}
            self.stuck_counter = 0
            self.suppression_mode = False
            self.approach_mode = False
            self.locked_angle = 0.0
            self.avoid_last_error = 0.0
            self.suppression_mode_pub.publish(Bool(data=False))
            self.current_target_fire = None
            self.target_lock_start_time = None
            self.escape_state = None

    def lidar_callback(self, msg):
        try:
            self.lidar_ranges = np.array(msg.ranges, dtype=np.float32)
            self.lidar_angle_increment = msg.angle_increment
            self.lidar_angle_min = msg.angle_min
            self.lidar_received = True
        except Exception as e:
            self.get_logger().error(f"LIDAR callback failed: {str(e)}")
            self.lidar_ranges = None
            self.lidar_angle_increment = None
            self.lidar_angle_min = None

    def odom_callback(self, msg):
        try:
            self.robot_pose = msg.pose.pose
            self.odom_received = True

            # Set starting position on first odometry reading
            if self.starting_position is None:
                self.starting_position = (self.robot_pose.position.x, self.robot_pose.position.y)
                self.get_logger().info(f"🏠 Set starting position: {self.starting_position}")

        except Exception as e:
            self.get_logger().error(f"Odometry callback failed: {str(e)}")
            self.robot_pose = None

    def normalize_angle(self, ang):
        return (ang + math.pi) % (2 * math.pi) - math.pi

    def quaternion_to_yaw(self, q):
        try:
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)
        except Exception:
            self.get_logger().error("Failed to compute yaw from quaternion")
            return 0.0

    def detect_circular_movement(self):
        if len(self.recent_actions) < 8:
            return False
        recent = list(self.recent_actions)[-8:]
        turn_left_count = recent.count(1)
        turn_right_count = recent.count(2)
        if (turn_left_count + turn_right_count) >= 6 and recent.count(0) <= 2:
            return True
        if len(recent) >= 4:
            first_half = recent[:4]
            second_half = recent[4:]
            if first_half == second_half:
                return True
        return False

    def get_avoidance_cmd(self):
        cmd = Twist()
        if self.lidar_ranges is None or len(self.lidar_ranges) == 0 or self.robot_pose is None:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - getattr(self, '_last_lidar_odom_warn', 0) >= 10.0:
                self.get_logger().warning("No LIDAR or odometry data, stopping robot")
                self._last_lidar_odom_warn = current_time
            return cmd

        ranges = np.array(self.lidar_ranges)
        ranges = np.where(np.isfinite(ranges), ranges, 10.0)
        ranges = np.where(ranges <= 0.0, 10.0, ranges)

        n = len(ranges)
        try:
            ang_min = float(self.lidar_angle_min)
            ang_inc = float(self.lidar_angle_increment)
            angles = ang_min + np.arange(n) * ang_inc
            angles = self.normalize_angle(angles)
            window_half = self.avoidance_window / 2.0
            front_mask = np.abs(angles) <= window_half
            front_indices = np.where(front_mask)[0]
        except Exception:
            self.get_logger().error("Error processing LIDAR angles")
            front_indices = []

        if len(front_indices) < self.min_obstacle_beams:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - getattr(self, '_last_no_obstacles_log', 0) >= 10.0:
                self.get_logger().debug("No obstacles in window, moving forward")
                self._last_no_obstacles_log = current_time
            return cmd

        yaw = self.quaternion_to_yaw(self.robot_pose.orientation) if self.robot_pose else 0.0
        fire_angle = self.current_obs[3] if self.current_obs is not None and len(self.current_obs) >= 4 else 0.0
        goal_angle = self.normalize_angle(yaw + fire_angle)

        window_size = int(self.avoidance_window / ang_inc)
        num_windows = 5
        window_centers = np.linspace(-window_half, window_half, num_windows)
        costs = []

        for center in window_centers:
            window_mask = np.abs(angles - center) <= window_half / num_windows
            window_ranges = ranges[window_mask]
            if len(window_ranges) == 0:
                costs.append(float('inf'))
                continue
            min_dist = np.min(window_ranges) if np.any(np.isfinite(window_ranges)) else 10.0
            angle_diff = abs(self.normalize_angle(center - goal_angle))
            cost = (self.path_cost_distance_weight * (1.0 / (min_dist + 0.1))) + (self.path_cost_obstacle_weight * angle_diff)
            costs.append(cost)

        if all(c == float('inf') for c in costs):
            cmd.linear.x = self.backward_speed
            cmd.angular.z = 0.0
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - getattr(self, '_last_no_paths_warn', 0) >= 10.0:
                self.get_logger().warning("No valid paths found, moving backward")
                self._last_no_paths_warn = current_time
            return cmd

        best_window_idx = np.argmin(costs)
        best_angle = window_centers[best_window_idx]
        min_dist = np.min(ranges[np.abs(angles - best_angle) <= window_half / num_windows])

        threshold = self.emergency_threshold * (self.approach_escape_scale if self.approach_mode else 1.0)
        if min_dist < threshold:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - getattr(self, '_last_emergency_warn', 0) >= 10.0:
                self.get_logger().warn(f"[EMERGENCY] Close to obstacle ({min_dist:.2f}m) - initiating escape")
                self._last_emergency_warn = current_time
            self.escape_state = 'backup'
            self.escape_end_time = self.get_clock().now().nanoseconds / 1e9 + (self.emergency_escape_duration * (self.approach_escape_scale if self.approach_mode else 1.0))
            self.escape_rotation_side = -self.turn_speed if best_angle < 0 else self.turn_speed
            cmd.linear.x = self.emergency_back_speed * (self.approach_escape_scale if self.approach_mode else 1.0)
            cmd.angular.z = 0.0
            return cmd

        cmd.linear.x = max(0.0, min(self.forward_speed, min_dist * 0.5))
        cmd.angular.z = max(-self.avoidance_max_angular, min(self.avoidance_max_angular, best_angle * self.avoid_kp))
        if min_dist < self.min_safe_distance * 1.5:
            cmd.linear.x = self.slow_forward_speed
        
        # Log avoidance path selection every 3 seconds to reduce spam
        current_time = self.get_clock().now().nanoseconds / 1e9
        if not hasattr(self, '_last_avoidance_log') or current_time - self._last_avoidance_log >= 3.0:
            self.get_logger().info(f"[AVOID] Selected path at angle {best_angle:.2f}rad ({np.degrees(best_angle):.1f}°), min_dist={min_dist:.2f}m, linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f}")
            self._last_avoidance_log = current_time
        
        return cmd

    def execute_escape(self):
        cmd = Twist()
        now = self.get_clock().now().nanoseconds / 1e9
        if self.escape_state == 'suppress' and now < self.escape_end_time:
            # Stop completely during fire suppression
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif self.escape_state == 'suppress':
            # Suppression time is up, start rotating
            self.escape_state = 'rotate'
            self.escape_end_time = now + 3.0  # Rotate for 3 seconds
            cmd.linear.x = 0.0
            cmd.angular.z = self.escape_rotation_side
        elif self.escape_state == 'backup' and now < self.escape_end_time:
            cmd.linear.x = self.emergency_back_speed * (self.approach_escape_scale if self.approach_mode else 1.0)
            cmd.angular.z = 0.0
        elif self.escape_state == 'backup':
            self.escape_state = 'rotate'
            self.escape_end_time = now + (self.emergency_escape_duration * 0.4)
            cmd.linear.x = 0.0
            cmd.angular.z = self.escape_rotation_side
        elif self.escape_state == 'rotate' and now < self.escape_end_time:
            cmd.linear.x = 0.0
            cmd.angular.z = self.escape_rotation_side
        else:
            self.escape_state = None
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        return cmd

    def select_rl_action(self):
        cmd = Twist()
        now = self.get_clock().now().nanoseconds / 1e9
        
        if self.current_obs is None:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - getattr(self, '_last_no_obs_warn', 0) >= 10.0:
                self.get_logger().warning("No observation, stopping robot")
                self._last_no_obs_warn = current_time
            return None, "stop", cmd

        min_obstacle_distance = self.current_obs[2]
        fire_detected = self.current_obs[0] > 0.5
        angle_to_fire = self.current_obs[3]
        fire_distance = self.current_obs[4]

        epsilon = self.epsilon_end + (self.epsilon_start - self.epsilon_end) * \
                  np.exp(-1.0 * self.step_count / self.epsilon_decay)

        # Add hysteresis to prevent rapid avoidance re-triggering
        avoidance_threshold = self.min_safe_distance * 1.2 if self.avoidance_count > 0 else self.min_safe_distance

        if min_obstacle_distance < avoidance_threshold:
            # Check if we've been in avoidance too long
            if self.avoidance_start_time is None:
                self.avoidance_start_time = now
                self.avoidance_count += 1
                # Log avoidance trigger every 3 seconds to reduce spam
                if not hasattr(self, '_last_avoidance_trigger_log') or now - self._last_avoidance_trigger_log >= 3.0:
                    self.get_logger().info(f"🔶 AVOIDANCE TRIGGERED: min_obstacle_distance={min_obstacle_distance:.2f}m < threshold={avoidance_threshold:.2f}m (hysteresis applied)")
                    self._last_avoidance_trigger_log = now
            elif now - self.avoidance_start_time > self.avoidance_timeout:
                # Timeout reached - temporarily increase threshold to escape
                # Log timeout every 3 seconds to reduce spam
                if not hasattr(self, '_last_timeout_log') or now - self._last_timeout_log >= 3.0:
                    self.get_logger().warning(f"⏰ AVOIDANCE TIMEOUT: Been avoiding for {now - self.avoidance_start_time:.1f}s, temporarily increasing threshold")
                    self._last_timeout_log = now
                self.avoidance_start_time = None
                # Force a move by using a higher threshold temporarily
                if min_obstacle_distance < self.min_safe_distance * 1.5:
                    self.get_logger().info(f"🚀 ESCAPING AVOIDANCE: min_obstacle_distance={min_obstacle_distance:.2f}m, using relaxed threshold")
                    cmd = self.get_avoidance_cmd()
                    if self.approach_mode:
                        self.post_avoid_recenter = True
                    return None, "escape_avoidance", cmd
                else:
                    self.get_logger().info(f"✅ AVOIDANCE CLEARED: min_obstacle_distance={min_obstacle_distance:.2f}m >= relaxed threshold")
                    self.avoidance_start_time = None
            else:
                if now - getattr(self, '_last_avoidance_continuing_log', 0) >= 10.0:
                    self.get_logger().debug(f"🔶 AVOIDANCE CONTINUING: {now - self.avoidance_start_time:.1f}s elapsed")
                    self._last_avoidance_continuing_log = now
            
            cmd = self.get_avoidance_cmd()
            if self.approach_mode:
                self.post_avoid_recenter = True
            return None, "avoidance", cmd
        
        if now - getattr(self, '_last_no_avoidance_log', 0) >= 10.0:
            self.get_logger().debug(f"✅ No avoidance needed: min_obstacle_distance={min_obstacle_distance:.2f}m >= threshold={avoidance_threshold:.2f}m")
            self._last_no_avoidance_log = now
        
        # Reset avoidance timer when not avoiding
        if self.avoidance_start_time is not None:
            self.get_logger().info(f"✅ AVOIDANCE CLEARED: Was avoiding for {(self.get_clock().now().nanoseconds / 1e9) - self.avoidance_start_time:.1f}s")
            self.avoidance_start_time = None
            # Reset avoidance count after successful clear
            if self.avoidance_count > 0:
                self.avoidance_count = max(0, self.avoidance_count - 1)
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.suppression_mode:
            action = self.navigation_action()
            if current_time - getattr(self, '_last_mode_log', 0) >= 3.0:
                self.get_logger().debug("🤖 SUPPRESSION MODE: navigating")
                self._last_mode_log = current_time
        elif self.approach_mode:
            action = self.navigation_action()
            if current_time - getattr(self, '_last_mode_log', 0) >= 10.0:
                self.get_logger().debug("🤖 APPROACH MODE: using navigation action")
                self._last_mode_log = current_time
        elif fire_detected:
            action = self.navigation_action()
            if current_time - getattr(self, '_last_mode_log', 0) >= 10.0:
                self.get_logger().debug("🤖 FIRE DETECTED: using navigation action")
                self._last_mode_log = current_time
        else:
            action = self.select_smart_action(epsilon)
            if current_time - getattr(self, '_last_mode_log', 0) >= 10.0:
                self.get_logger().debug("🤖 NO FIRE: using DQN action")
                self._last_mode_log = current_time

        if action is not None:
            action_name = self.action_map[action]
            self.recent_actions.append(action)
            self.action_counts[action] = self.action_counts.get(action, 0) + 1
            cmd = self.execute_action(action_name, action)
        return action, action_name if action is not None else "avoidance", cmd

    def navigation_action(self):
        angle_to_fire = self.current_obs[3]
        fire_distance = self.current_obs[4]
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - getattr(self, '_last_navigation_log', 0) >= 10.0:
            self.get_logger().debug(f"🧭 Navigation: angle_to_fire={angle_to_fire:.3f}rad ({math.degrees(angle_to_fire):.1f}°), threshold={self.angle_threshold:.3f}rad ({math.degrees(self.angle_threshold):.1f}°), fire_dist={fire_distance:.2f}m")
            self._last_navigation_log = current_time
        
        if abs(angle_to_fire) > self.angle_threshold:
            if angle_to_fire > 0:
                if current_time - getattr(self, '_last_navigation_log', 0) >= 3.0:
                    self.get_logger().debug("🧭 Turning RIGHT toward fire")
                    self._last_navigation_log = current_time
                return 2  # turn_right
            else:
                if current_time - getattr(self, '_last_navigation_log', 0) >= 3.0:
                    self.get_logger().debug("🧭 Turning LEFT toward fire")
                    self._last_navigation_log = current_time
                return 1  # turn_left
        else:
            action = 5 if fire_distance < self.min_safe_distance * 2.0 else 0  # slow_forward or move_forward
            if current_time - getattr(self, '_last_navigation_log', 0) >= 3.0:
                self.get_logger().debug(f"🧭 Moving FORWARD toward fire (action={action})")
                self._last_navigation_log = current_time
            return action

    def select_smart_action(self, epsilon):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Check if we've been spinning too long
        if self.last_action_time is not None:
            time_since_last_action = current_time - self.last_action_time
            if time_since_last_action > self.spin_timeout:
                self.get_logger().info("🛑 Spin timeout reached, switching to search mode")
                self.search_mode = True
                self.search_start_time = current_time
        
        # If in search mode, prefer forward movement
        if self.search_mode:
            # Exit search mode after 30 seconds or if we detect obstacles
            if (self.search_start_time and current_time - self.search_start_time > 30.0) or \
               (self.current_obs is not None and self.current_obs[2] < self.min_safe_distance):
                self.search_mode = False
                self.search_start_time = None
                self.get_logger().info("🔍 Exiting search mode")
            
            # In search mode, prefer forward actions
            if np.random.random() < 0.7:  # 70% chance to move forward
                return 0  # move_forward
            else:
                # Occasionally turn to search different areas
                return np.random.choice([1, 2])  # turn_left or turn_right
        
        # Normal DQN action selection
        if np.random.random() < epsilon:
            if self.detect_circular_movement():
                non_forward_actions = [a for a in range(self.action_size) if a not in [0, 5]]
                counts = [self.action_counts.get(a, 0) for a in non_forward_actions]
                min_count = min(counts) if counts else 0
                candidates = [a for a, c in zip(non_forward_actions, counts) if c == min_count]
                return int(np.random.choice(candidates))
            weights = np.ones(self.action_size)
            for action, count in self.action_counts.items():
                if count > 5:
                    weights[action] *= 0.5
            weights /= weights.sum()
            return int(np.random.choice(self.action_size, p=weights))
        else:
            state_tensor = torch.FloatTensor(self.current_obs).unsqueeze(0).to(self.agent.device)
            with torch.no_grad():
                q_values = self.agent.policy_net(state_tensor)
            q_values_adj = q_values.clone()
            for action, count in self.action_counts.items():
                if count > 10:
                    q_values_adj[0][action] -= 0.1
            return q_values_adj.argmax().item()

    def execute_action(self, action_name, action_id):
        self.last_action_time = self.get_clock().now().nanoseconds / 1e9
        
        cmd = Twist()
        if action_name == "move_forward":
            obstacle_dist = self.current_obs[2] if self.current_obs is not None else self.min_safe_distance
            base_speed = min(0.8, max(0.15, obstacle_dist * 0.3))
            speed = max(0.0, min(0.8, base_speed))
            cmd.linear.x = speed
            cmd.angular.z = 0.0
        elif action_name == "turn_left":
            cmd.linear.x = 0.05
            cmd.angular.z = -self.turn_speed  # Fixed: negative for left turn
        elif action_name == "turn_right":
            cmd.linear.x = 0.05
            cmd.angular.z = self.turn_speed   # Fixed: positive for right turn
        elif action_name == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif action_name == "move_backward":
            cmd.linear.x = self.backward_speed
            cmd.angular.z = 0.0
        elif action_name == "slow_forward":
            cmd.linear.x = self.slow_forward_speed
            cmd.angular.z = 0.0
        return cmd

    def get_stop_cmd(self):
        """Get a stop command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd

    def check_return_to_base(self):
        """Check if robot should return to base due to no NEW fire detection for timeout period"""
        if not self.is_ready or self.return_to_base_mode:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # If we haven't detected any NEW fire yet, don't start the timer
        if self.last_new_fire_detection_time is None:
            return

        # Check if no NEW fire has been detected for the timeout period
        time_since_last_new_fire = current_time - self.last_new_fire_detection_time
        if time_since_last_new_fire > self.no_fire_timeout:
            if not self.return_to_base_mode:
                self.return_to_base_mode = True
                self.return_to_base_start_time = current_time
                self.reset_return_navigation()  # Reset navigation variables
                self.get_logger().error(f"🚨 NO NEW FIRES FOR {self.no_fire_timeout} SECONDS - INITIATING RETURN TO BASE!")
                # Cancel the timer to avoid repeated calls
                if hasattr(self, 'return_to_base_timer'):
                    self.return_to_base_timer.cancel()
        else:
            # Log remaining time for debugging
            remaining_time = self.no_fire_timeout - time_since_last_new_fire
            if remaining_time < 10:  # Only log when less than 10 seconds remaining
                if current_time - getattr(self, '_last_return_to_base_log', 0) >= 10.0:
                    self.get_logger().debug(f"🏠 Return to base in {remaining_time:.1f} seconds")
                    self._last_return_to_base_log = current_time

    def debug_return_to_base_status(self):
        """Debug method to log return-to-base timing information"""
        if not self.is_ready:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.return_to_base_mode and self.return_to_base_start_time is not None:
            time_in_return_mode = current_time - self.return_to_base_start_time
            self.get_logger().info(f"🏠 RETURN TO BASE ACTIVE: {time_in_return_mode:.1f} seconds navigating")
            
            # Log current position and target
            if self.robot_pose is not None and self.starting_position is not None:
                current_pos = (self.robot_pose.position.x, self.robot_pose.position.y)
                target_pos = self.starting_position
                distance = math.sqrt((target_pos[0] - current_pos[0])**2 + (target_pos[1] - current_pos[1])**2)
                self.get_logger().info(f"📍 Position: {current_pos}, Target: {target_pos}, Distance: {distance:.2f}m")
        
        if self.last_new_fire_detection_time is not None:
            time_since_last_new_fire = current_time - self.last_new_fire_detection_time
            remaining_time = max(0, self.no_fire_timeout - time_since_last_new_fire)
            
            # Show prominent countdown when close to return-to-base
            if remaining_time <= 10 and remaining_time > 0:
                current_time_check = self.get_clock().now().nanoseconds / 1e9
                if current_time_check - getattr(self, '_last_return_countdown_warn', 0) >= 10.0:
                    self.get_logger().warn(f"⏰ RETURN TO BASE IN {remaining_time:.1f} SECONDS!")
                    self._last_return_countdown_warn = current_time_check
            elif remaining_time == 0:
                self.get_logger().error("� RETURN TO BASE TIMER EXPIRED - INITIATING RETURN!")
            else:
                self.get_logger().info(f"🔥 Last NEW fire: {time_since_last_new_fire:.1f}s ago, return-to-base in {remaining_time:.1f}s")
        else:
            self.get_logger().info(f"🔥 No new fires detected yet")
            
        # Add search mode and spinning information
        if self.search_mode:
            search_duration = current_time - self.search_start_time if self.search_start_time else 0
            self.get_logger().info(f"🔍 In search mode for {search_duration:.1f}s")
        
        if self.last_action_time is not None:
            time_since_action = current_time - self.last_action_time
            if time_since_action > 5.0:
                self.get_logger().info(f"⚠️  No action taken for {time_since_action:.1f}s")
        
        self.get_logger().info(f"📋 Suppressed fires: {len(self.suppressed_fire_positions)}, Current target: {self.current_target_fire}")
        self.get_logger().info(f"🤖 Mode: Approach={self.approach_mode}, Suppression={self.suppression_mode}, Return-to-base={self.return_to_base_mode}, Mission-Complete={self.mission_complete}")

    def get_fire_detection_info(self):
        """Get detailed information about fire detection timing"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        info = {
            'current_time': current_time,
            'last_new_fire_time': self.last_new_fire_detection_time,
            'last_any_fire_time': self.last_fire_detection_time,
            'return_to_base_mode': self.return_to_base_mode,
            'return_to_base_start': self.return_to_base_start_time,
            'search_mode': self.search_mode,
            'suppressed_fires_count': len(self.suppressed_fire_positions),
            'current_target': self.current_target_fire
        }
        
        if self.last_new_fire_detection_time is not None:
            info['time_since_last_new_fire'] = current_time - self.last_new_fire_detection_time
            info['remaining_to_return'] = max(0, self.no_fire_timeout - info['time_since_last_new_fire'])
        else:
            info['time_since_last_new_fire'] = None
            info['remaining_to_return'] = None
            
        return info

    def reset_search_mode(self):
        """Reset search mode and spinning detection"""
        self.search_mode = False
        self.search_start_time = None
        self.last_action_time = None
        self.get_logger().info("🔄 Search mode and spinning detection reset")

    def reset_return_navigation(self):
        """Reset return-to-base navigation variables"""
        self.last_return_position = None
        self.stuck_counter = 0
        self.rotation_start_time = None
        self.get_logger().info("🔄 Return-to-base navigation variables reset")

    def reset_mission(self):
        """Reset mission complete flag to allow robot to start moving again"""
        self.mission_complete = False
        self.get_logger().info("🔄 Mission reset - robot can now move again")

    def force_return_to_base(self):
        """Force the robot to return to base immediately (for testing)"""
        if self.starting_position is None:
            self.get_logger().error("Cannot force return to base: no starting position set")
            return False
        
        self.return_to_base_mode = True
        self.return_to_base_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().error("🚨 FORCED RETURN TO BASE ACTIVATED!")
        return True

    def get_fire_detection_info(self):
        """Get comprehensive fire detection status information"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Calculate time since last new fire detection
        last_new_fire_time = self.last_new_fire_detection_time if self.last_new_fire_detection_time else 0.0
        time_since_last_new_fire = current_time - last_new_fire_time if self.last_new_fire_detection_time else 0.0
        
        # Calculate remaining time to return to base
        remaining_to_return = 0.0
        if self.last_new_fire_detection_time and not self.return_to_base_mode:
            remaining_to_return = max(0, self.no_fire_timeout - time_since_last_new_fire)
        
        # Get current target information
        current_target = str(self.current_target_fire) if self.current_target_fire else "None"
        
        return {
            'current_time': current_time,
            'last_new_fire_time': last_new_fire_time,
            'time_since_last_new_fire': time_since_last_new_fire,
            'return_to_base_mode': self.return_to_base_mode,
            'remaining_to_return': remaining_to_return,
            'search_mode': self.search_mode,
            'suppressed_fires_count': len(self.suppressed_fire_positions),
            'current_target': current_target
        }

    def handle_fire_info_request(self, request, response):
        """Handle service request for fire detection information"""
        info = self.get_fire_detection_info()
        
        response.success = True
        response.message = f"""🔥 FIRE DETECTION STATUS:
Current Time: {info['current_time']:.1f}s
Last New Fire: {info['last_new_fire_time']:.1f}s ago ({info['time_since_last_new_fire']:.1f}s)
Return-to-Base: {info['return_to_base_mode']} (remaining: {info['remaining_to_return']:.1f}s)
Search Mode: {info['search_mode']}
Suppressed Fires: {info['suppressed_fires_count']}
Current Target: {info['current_target']}
Mission Complete: {self.mission_complete}"""
        
        return response

    def handle_reset_mission_request(self, request, response):
        """Handle service request to reset mission complete flag"""
        self.reset_mission()
        response.success = True
        response.message = "Mission reset successfully - robot can now move again"
        return response

    def update_fire_detection_time(self):
        """Update the last fire detection timestamp"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.last_fire_detection_time = current_time

    def update_new_fire_detection_time(self):
        """Update the last NEW fire detection timestamp (only for non-suppressed fires)"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.last_new_fire_detection_time = current_time
        self.get_logger().info(f"🔥 NEW fire detected at {current_time:.1f}s - resetting return-to-base timer")

    def return_to_base(self):
        """Navigate back to the starting position using deterministic control (no RL)"""
        cmd = Twist()  # Initialize cmd at the beginning
        
        self.get_logger().info("🏠 RETURN_TO_BASE METHOD CALLED")
        
        if self.starting_position is None or self.robot_pose is None:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - getattr(self, '_last_return_base_warn', 0) >= 10.0:
                self.get_logger().warning("Cannot return to base: missing starting position or current pose")
                self._last_return_base_warn = current_time
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd

        # Calculate the vector from current position to starting position
        current_x = self.robot_pose.position.x
        current_y = self.robot_pose.position.y
        start_x, start_y = self.starting_position

        # Calculate distance and angle to starting position
        dx = start_x - current_x
        dy = start_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        current_position = (current_x, current_y)
        
        # Handle case where we're already at the base
        if distance < 0.1:
            self.get_logger().error("🏠 REACHED BASE! Robot stopped at starting position!")
            self.return_to_base_mode = False
            self.return_to_base_start_time = None
            self.mission_complete = True  # Mission is now complete
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("🤖 Robot stopped - mission complete!")
            return cmd

        # Check if robot is stuck (not moving)
        if self.last_return_position is not None:
            dist_moved = math.sqrt((current_x - self.last_return_position[0])**2 +
                                 (current_y - self.last_return_position[1])**2)
            if dist_moved < 0.05:  # Less than 5cm movement
                # Only count as stuck if we're not intentionally rotating
                # Check the last command sent to see if we were trying to move
                if hasattr(self, '_last_cmd_linear') and self._last_cmd_linear > 0.01:
                    # We were trying to move forward but didn't move - this is stuck
                    self.stuck_counter += 1
                    self.get_logger().warn(f"🏠 STUCK DETECTED: Trying to move forward (linear.x={self._last_cmd_linear:.2f}) but only moved {dist_moved:.3f}m. Counter: {self.stuck_counter}")
                elif hasattr(self, '_last_cmd_angular') and abs(self._last_cmd_angular) > 0.01:
                    # We were rotating - this is normal, don't count as stuck
                    old_counter = self.stuck_counter
                    self.stuck_counter = max(0, self.stuck_counter - 1)  # Decrease counter
                    self.get_logger().debug(f"🏠 ROTATING: Angular command {self._last_cmd_angular:.2f}, moved {dist_moved:.3f}m. Counter: {old_counter} -> {self.stuck_counter}")
                else:
                    # No movement command - might be stuck
                    self.stuck_counter += 1
                    self.get_logger().warn(f"🏠 STUCK DETECTED: No movement command but only moved {dist_moved:.3f}m. Counter: {self.stuck_counter}")
                
                if self.stuck_counter > 20:  # Stuck for 2 seconds (10Hz * 2)
                    # Check if there are obstacles preventing movement
                    if self.lidar_ranges is not None and len(self.lidar_ranges) > 0:
                        min_obstacle_distance = float(np.min(self.lidar_ranges))
                        if min_obstacle_distance < self.min_safe_distance:
                            current_time = self.get_clock().now().nanoseconds / 1e9
                            if current_time - getattr(self, '_last_stuck_obstacle_warn', 0) >= 10.0:
                                self.get_logger().warn("🏠 STUCK! Detected obstacle, trying avoidance...")
                                self._last_stuck_obstacle_warn = current_time
                            # Try obstacle avoidance instead of ending mission
                            avoidance_cmd = self.get_avoidance_cmd()
                            if avoidance_cmd.linear.x != 0.0 or avoidance_cmd.angular.z != 0.0:
                                self.get_logger().info("🏠 Using obstacle avoidance to navigate around blockage")
                                self.stuck_counter = max(0, self.stuck_counter - 10)  # Reset counter partially
                                return avoidance_cmd

                    # If no obstacles or avoidance failed, try a different approach
                    if self.stuck_counter > 40:  # Been stuck for 4 seconds total
                        self.get_logger().error("🏠 STUCK! No obstacles detected but cannot move, continuing to try...")
                        self.stuck_counter = 20  # Reset counter to keep trying rotation
                        # Try rotating to find a clear path
                        cmd.linear.x = 0.0
                        cmd.angular.z = self.turn_speed * 0.5  # Slow rotation
                        return cmd
                    else:
                        current_time = self.get_clock().now().nanoseconds / 1e9
                        if current_time - getattr(self, '_last_stuck_rotation_warn', 0) >= 10.0:
                            self.get_logger().warn("🏠 STUCK! No clear obstacles, trying rotation...")
                            self._last_stuck_rotation_warn = current_time
                        # Try rotating to find a clear path
                        cmd.linear.x = 0.0
                        cmd.angular.z = self.turn_speed * 0.5  # Slow rotation
                        return cmd
            else:
                # We're moving - reset stuck counter
                if self.stuck_counter > 0:
                    self.get_logger().info(f"🏠 MOVEMENT DETECTED: Moved {dist_moved:.3f}m, resetting stuck counter from {self.stuck_counter} to 0")
                self.stuck_counter = 0
        
        self.last_return_position = current_position

        # Calculate target angle
        angle_to_target = math.atan2(dy, dx)
        
        # Get current robot orientation
        current_yaw = self.quaternion_to_yaw(self.robot_pose.orientation)
        
        # Calculate angle difference and normalize it properly
        angle_diff = angle_to_target - current_yaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        # Debug logging for angle calculations
        self.get_logger().info(f"🏠 DEBUG: current_pos=({current_x:.2f}, {current_y:.2f}), target=(0, 0)")
        self.get_logger().info(f"🏠 DEBUG: dx={dx:.2f}, dy={dy:.2f}, angle_to_target={math.degrees(angle_to_target):.1f}°")
        self.get_logger().info(f"🏠 DEBUG: current_yaw={math.degrees(current_yaw):.1f}°, angle_diff={math.degrees(angle_diff):.1f}°")

        # If we're very close, just move directly to base (ignore angle)
        if distance < 0.5:
            cmd.linear.x = min(self.slow_forward_speed * 2, distance * 2)  # Move slowly when close
            cmd.angular.z = angle_diff * 0.3  # Small angular correction
            self.get_logger().info(f"🏠 FINAL APPROACH: distance={distance:.2f}m, angle_diff={math.degrees(angle_diff):.1f}°")
            return cmd

        # First, rotate towards the target if angle difference is significant
        if abs(angle_diff) > math.radians(15):  # 15 degrees threshold
            cmd.linear.x = 0.0
            # Choose rotation direction and speed based on angle magnitude
            rotation_speed = min(self.turn_speed, abs(angle_diff) * 2.0)  # Faster for larger angles
            cmd.angular.z = rotation_speed if angle_diff > 0 else -rotation_speed
            
            # Track rotation time to prevent infinite spinning
            if self.rotation_start_time is None:
                self.rotation_start_time = self.get_clock().now().nanoseconds / 1e9
            elif (self.get_clock().now().nanoseconds / 1e9 - self.rotation_start_time) > 30:  # 30 second timeout
                self.get_logger().error("🏠 ROTATION TIMEOUT! Moving forward anyway")
                cmd.linear.x = self.slow_forward_speed
                cmd.angular.z = 0.0
                self.rotation_start_time = None
            
            self.get_logger().info(f"🏠 ROTATING: angle_diff={math.degrees(angle_diff):.1f}°, distance={distance:.2f}m, cmd.angular.z={cmd.angular.z:.2f}")
        else:
            # Clear rotation timeout when we start moving
            self.rotation_start_time = None
            
            # Move towards the target with small angular corrections
            cmd.linear.x = min(self.forward_speed * 0.8, distance)  # Slightly slower than normal
            cmd.angular.z = angle_diff * 0.5  # Proportional control for small corrections
            self.get_logger().info(f"🏠 MOVING: distance={distance:.2f}m, speed={cmd.linear.x:.2f}, correction={cmd.angular.z:.2f}")

        return cmd

    def control_loop(self):
        # Check if mission is complete - robot should stay stopped
        if self.mission_complete:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        if not self.is_ready:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - getattr(self, '_last_not_ready_warn', 0) >= 10.0:
                self.get_logger().warning("Node not ready, skipping control loop")
                self._last_not_ready_warn = current_time
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        if self.current_obs is None:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - getattr(self, '_last_no_obs_control_warn', 0) >= 10.0:
                self.get_logger().warning("No observation received, skipping control loop")
                self._last_no_obs_control_warn = current_time
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        if self.escape_state:
            cmd = self.execute_escape()
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"[ESCAPE] State: {self.escape_state}, linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f}")
            return

        # Check if we should return to base
        if self.return_to_base_mode:
            if self.starting_position is None:
                self.get_logger().error("🏠 Cannot return to base: no starting position recorded!")
                self.return_to_base_mode = False
                self.mission_complete = True  # End mission if we can't return
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                return cmd
            
            cmd = self.return_to_base()
            # Track the command for stuck detection
            self._last_cmd_linear = cmd.linear.x
            self._last_cmd_angular = cmd.angular.z
            self.cmd_pub.publish(cmd)
            # Log return-to-base status every second
            current_time = self.get_clock().now().nanoseconds / 1e9
            if hasattr(self, '_last_return_log') and current_time - self._last_return_log > 1.0:
                distance_to_base = math.sqrt((self.starting_position[0] - self.robot_pose.position.x)**2 + 
                                           (self.starting_position[1] - self.robot_pose.position.y)**2)
                self.get_logger().warn(f"🏠 RETURNING TO BASE: {distance_to_base:.2f}m remaining")
                self._last_return_log = current_time
            elif not hasattr(self, '_last_return_log'):
                self._last_return_log = current_time
            return

        fire_detected = self.current_obs[0] > 0.6
        self.fire_detected_buffer.append(fire_detected)
        stable_fire_detected = sum(self.fire_detected_buffer) >= 3

        # Debug fire detection status
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - getattr(self, '_last_fire_detection_log', 0) >= 10.0:
            buffer_str = str(list(self.fire_detected_buffer)[-5:]) if len(self.fire_detected_buffer) >= 5 else str(list(self.fire_detected_buffer))
            detection_status = "RESPONSIVE" if fire_detected else "QUIET"
            self.get_logger().debug(f"🔥 Fire detection [{detection_status}]: detected={fire_detected}, stable={stable_fire_detected}, buffer={buffer_str}")
            self.get_logger().debug(f"📊 Observation: fire_prob={self.current_obs[0]:.3f}, fire_size={self.current_obs[1]:.3f}, min_dist={self.current_obs[2]:.3f}, angle={self.current_obs[3]:.3f}, fire_dist={self.current_obs[4]:.3f}")
            self._last_fire_detection_log = current_time

        # Show countdown timer for return-to-base
        if self.last_new_fire_detection_time is not None and not self.return_to_base_mode:
            current_time = self.get_clock().now().nanoseconds / 1e9
            time_since_last_new_fire = current_time - self.last_new_fire_detection_time
            remaining_time = max(0, self.no_fire_timeout - time_since_last_new_fire)
            
            # Show countdown every 5 seconds when more than 10 seconds remaining
            if remaining_time > 10 and int(remaining_time) % 5 == 0:
                if not hasattr(self, '_last_countdown') or self._last_countdown != int(remaining_time):
                    self.get_logger().info(f"⏰ Return to base in {int(remaining_time)} seconds")
                    self._last_countdown = int(remaining_time)
            # Show urgent countdown when less than 10 seconds
            elif 0 < remaining_time <= 10:
                if not hasattr(self, '_last_urgent') or self._last_urgent != int(remaining_time):
                    current_time = self.get_clock().now().nanoseconds / 1e9
                    if current_time - getattr(self, '_last_urgent_warn', 0) >= 10.0:
                        self.get_logger().warn(f"🚨 RETURN TO BASE IN {int(remaining_time)} SECONDS!")
                        self._last_urgent_warn = current_time
                    self._last_urgent = int(remaining_time)

        fire_distance = self.current_obs[4] if len(self.current_obs) >= 5 else 999.0

        # Calculate current fire position in world coordinates using polar to cartesian conversion
        current_fire_position = None
        if len(self.current_obs) >= 5 and self.robot_pose is not None:
            angle_to_fire = self.current_obs[3]  # Angle in radians
            fire_dist = self.current_obs[4]      # Distance in meters

            # Validate inputs to prevent RuntimeWarning
            if not (math.isfinite(angle_to_fire) and math.isfinite(fire_dist) and fire_dist >= 0):
                self.get_logger().debug(f"Invalid fire detection data: angle={angle_to_fire}, dist={fire_dist}")
            else:
                # Convert polar coordinates to Cartesian relative to robot
                fire_x_relative = fire_dist * math.cos(angle_to_fire)
                fire_y_relative = fire_dist * math.sin(angle_to_fire)

                # Convert to world coordinates using robot's current position and orientation
                robot_x = self.robot_pose.position.x
                robot_y = self.robot_pose.position.y
                robot_yaw = self.quaternion_to_yaw(self.robot_pose.orientation)

                # Rotate relative coordinates by robot's orientation and add to robot position
                fire_x_world = robot_x + fire_x_relative * math.cos(robot_yaw) - fire_y_relative * math.sin(robot_yaw)
                fire_y_world = robot_y + fire_x_relative * math.sin(robot_yaw) + fire_y_relative * math.cos(robot_yaw)

                current_fire_position = (fire_x_world, fire_y_world)

        # Update fire detection time if fire is detected
        if stable_fire_detected:
            # Check if this is a NEW fire (not suppressed)
            is_new_fire = True
            if current_fire_position:
                for suppressed_pos in self.suppressed_fire_positions:
                    if suppressed_pos and math.dist(suppressed_pos, current_fire_position) < 0.2:
                        is_new_fire = False
                        break
            
            if is_new_fire:
                self.update_new_fire_detection_time()
                # If we were returning to base but detected a new fire, cancel return-to-base
                if self.return_to_base_mode:
                    self.return_to_base_mode = False
                    self.return_to_base_start_time = None
                    self.get_logger().info("🔥 NEW fire detected, canceling return to base")
            else:
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - getattr(self, '_last_suppressed_fire_log', 0) >= 3.0:
                    self.get_logger().debug(f"🔥 Detected suppressed fire at {current_fire_position}, not resetting return-to-base timer")
                    self._last_suppressed_fire_log = current_time

        current_time = self.get_clock().now().nanoseconds / 1e9
        if (fire_detected and not self.suppression_mode and not self.approach_mode and
            current_fire_position and current_fire_position not in self.suppressed_fire_positions):
            if self.current_target_fire is None:
                self.current_target_fire = current_fire_position
                self.target_lock_start_time = current_time
                stability_status = "stable" if stable_fire_detected else "unstable"
                self.get_logger().info(f"🔒 Locked onto new target fire at {self.current_target_fire} ({stability_status})")
            else:
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - getattr(self, '_last_focused_fire_log', 0) >= 10.0:
                    self.get_logger().debug(
                        f"Detected fire at {current_fire_position} but focused on {self.current_target_fire}; ignoring"
                    )
                    self._last_focused_fire_log = current_time

        def is_suppressed(pos):
            for s in self.suppressed_fire_positions:
                if s and pos and math.dist(s, pos) < 0.2:
                    return True
            return False
        if current_fire_position and is_suppressed(current_fire_position):
            stable_fire_detected = False

        if self.approach_mode and fire_distance < self.suppression_distance:
            # No recentering in approach mode anymore
            pass
        else:
            self.last_recenter_distance = None
            self.recenter_checkpoints = []

        if stable_fire_detected and not self.suppression_mode and not self.approach_mode:
            self.get_logger().info("🔥 Stable fire detected, entering approach mode")
            self.approach_mode = True
            self.locked_angle = self.current_obs[3]
            self.suppression_mode_pub.publish(Bool(data=False))
            self.vla_pub.publish(Bool(data=False))
            self.approach_lost_since = None
        elif not stable_fire_detected and fire_detected and not self.suppression_mode and not self.approach_mode:
            # Respond to unstable fire detection - might become stable
            self.get_logger().info("🔥 Unstable fire detected, entering approach mode (hoping it becomes stable)")
            self.approach_mode = True
            self.locked_angle = self.current_obs[3]
            self.suppression_mode_pub.publish(Bool(data=False))
            self.vla_pub.publish(Bool(data=False))
            self.approach_lost_since = None
        elif not fire_detected and not self.suppression_mode and not self.approach_mode:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - getattr(self, '_last_no_fire_log', 0) >= 10.0:
                self.get_logger().debug(f"🔥 No fire detected: detected={fire_detected}, stable={stable_fire_detected}, buffer_sum={sum(self.fire_detected_buffer)}")
                self._last_no_fire_log = current_time
        elif self.approach_mode:
            if stable_fire_detected:
                self.get_logger().info("🟢 Fire is stable, entering suppression mode")
                self.approach_mode = False
                self.suppression_mode = True
                self.suppression_mode_pub.publish(Bool(data=True))
                self.vla_pub.publish(Bool(data=False))
                return
            elif not fire_detected:
                # Only start grace timer if no fire detected at all
                now = self.get_clock().now().nanoseconds / 1e9
                if self.approach_lost_since is None:
                    self.approach_lost_since = now
                    if now - getattr(self, '_last_approach_lost_log', 0) >= 10.0:
                        self.get_logger().debug(f"Approach detection lost, starting grace timer")
                        self._last_approach_lost_log = now
                elif now - self.approach_lost_since < self.approach_interrupt_timeout:
                    if now - getattr(self, '_last_approach_grace_log', 0) >= 10.0:
                        self.get_logger().debug(f"Approach within grace period: {now - self.approach_lost_since:.2f}s")
                        self._last_approach_grace_log = now
                else:
                    self.get_logger().info("🟢 Fire lost during approach, exiting approach mode")
                    self.approach_mode = False
                    self.suppression_mode = False
                    self.suppression_mode_pub.publish(Bool(data=False))
                    self.vla_pub.publish(Bool(data=False))
                    self.approach_lost_since = None
            else:
                # Fire detected but not stable - reset grace timer to be more tolerant
                self.approach_lost_since = None
        elif self.suppression_mode:
            if not stable_fire_detected:
                self.get_logger().info("🟢 Fire lost, exiting suppression mode")
                self.suppression_mode = False
                self.suppression_mode_pub.publish(Bool(data=False))
                self.vla_pub.publish(Bool(data=False))
                self.last_recenter_distance = None
                self.recenter_checkpoints = []
            elif fire_distance <= self.suppression_distance:
                self.get_logger().info("🟢 Fire reached suppression distance, stopping for 3 seconds to suppress")
                self.escape_state = 'suppress'
                self.escape_end_time = self.get_clock().now().nanoseconds / 1e9 + 3.0
                # Randomly choose rotation direction after suppression
                self.escape_rotation_side = self.turn_speed if np.random.random() > 0.5 else -self.turn_speed
                self.vla_pub.publish(Bool(data=True))
                # Add suppressed fire to the list if not already there
                if current_fire_position and current_fire_position not in self.suppressed_fire_positions:
                    self.suppressed_fire_positions.append(current_fire_position)
                    self.get_logger().info(f"📌 Recorded suppressed fire at {current_fire_position}")
                self.approach_mode = False
                self.suppression_mode = False
                self.suppression_mode_pub.publish(Bool(data=False))
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0  # Stop completely during suppression
                self.cmd_pub.publish(cmd)
                self.last_recenter_distance = None
                self.recenter_checkpoints = []
                return
            elif fire_distance > self.suppression_distance:
                # Recentering logic in suppression mode
                if self.last_recenter_distance is None:
                    self.last_recenter_distance = fire_distance
                    self.recenter_checkpoints = [fire_distance * (2/3), fire_distance * (1/3)]
                if self.recenter_checkpoints and fire_distance <= self.recenter_checkpoints[0]:
                    # Log suppression recentering every 3 seconds to reduce spam
                    current_time = self.get_clock().now().nanoseconds / 1e9
                    if not hasattr(self, '_last_suppression_recenter_log') or current_time - self._last_suppression_recenter_log >= 3.0:
                        self.get_logger().info(f"🔄 Recentering to fire at distance {fire_distance:.2f}")
                        self._last_suppression_recenter_log = current_time
                    angle_to_fire = self.current_obs[3]
                    if abs(angle_to_fire) > self.angle_threshold:
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = (self.turn_speed * 0.5) if angle_to_fire > 0 else -(self.turn_speed * 0.5)
                        self.cmd_pub.publish(twist)
                        return
                    self.recenter_checkpoints.pop(0)
                # Continue navigating towards fire

        action, action_name, cmd = self.select_rl_action()
        
        # Add search mode indicator to action name
        if self.search_mode:
            action_name = f"SEARCH_{action_name}"
        
        if self.post_avoid_recenter and (self.lidar_ranges is None or float(np.min(self.lidar_ranges)) >= self.min_safe_distance):
            # Check for recentering timeout (5 seconds max)
            current_time = self.get_clock().now().nanoseconds / 1e9
            if not hasattr(self, 'recenter_start_time'):
                self.recenter_start_time = current_time
            elif current_time - self.recenter_start_time > 5.0:
                current_time_check = self.get_clock().now().nanoseconds / 1e9
                if current_time_check - getattr(self, '_last_recenter_timeout_warn', 0) >= 10.0:
                    self.get_logger().warn("⏰ RECENTERING TIMEOUT: Giving up recentering after 5 seconds")
                    self._last_recenter_timeout_warn = current_time_check
                self.post_avoid_recenter = False
                delattr(self, 'recenter_start_time')
                return

            angle_to_fire = self.current_obs[3] if (self.current_obs is not None and len(self.current_obs) >= 4) else 0.0
            if abs(angle_to_fire) > self.angle_threshold:
                rec_cmd = Twist()
                rec_cmd.linear.x = 0.0
                rec_cmd.angular.z = -self.turn_speed if angle_to_fire > 0 else self.turn_speed
                self.cmd_pub.publish(rec_cmd)
                # Log recentering every 3 seconds to reduce spam
                if not hasattr(self, '_last_recenter_log') or current_time - self._last_recenter_log >= 3.0:
                    self.get_logger().info(f"↪ Recentering toward fire: angle={angle_to_fire:.2f}")
                    self._last_recenter_log = current_time
                return
            else:
                self.post_avoid_recenter = False
                if hasattr(self, 'recenter_start_time'):
                    delattr(self, 'recenter_start_time')

        self.cmd_pub.publish(cmd)
        
        # Log action status every 3 seconds to reduce spam
        current_time = self.get_clock().now().nanoseconds / 1e9
        if not hasattr(self, '_last_action_log') or current_time - self._last_action_log >= 3.0:
            self.get_logger().info(
                f"🤖 Action: {action_name} | linear.x: {cmd.linear.x:.2f} | angular.z: {cmd.angular.z:.2f}"
            )
            self._last_action_log = current_time

        if self.previous_obs is not None:
            adjusted_reward = self.reward
            if self.detect_circular_movement():
                adjusted_reward += self.action_repeat_penalty
                self.stuck_counter += 1
                if self.stuck_counter % 10 == 0:
                    current_time = self.get_clock().now().nanoseconds / 1e9
                    if current_time - getattr(self, '_last_circular_warn', 0) >= 10.0:
                        self.get_logger().warn("🔄 Circular movement detected - applying penalty")
                        self._last_circular_warn = current_time
            else:
                self.stuck_counter = max(0, self.stuck_counter - 1)

            if self.mode != 'inference':
                if action is not None and 0 <= action < self.action_size:
                    self.agent.memory.push(self.previous_obs, action, adjusted_reward, self.current_obs, self.done)
                else:
                    current_time = self.get_clock().now().nanoseconds / 1e9
                    if current_time - getattr(self, '_last_invalid_action_log', 0) >= 10.0:
                        self.get_logger().debug(f"Invalid action index: {action}, skipping memory push.")
                        self._last_invalid_action_log = current_time

            if self.mode == 'train_online':
                loss = self.agent.train_step()
                self.step_count += 1

                if self.step_count % self.save_interval == 0:
                    saved_path = self.save_model()
                    self.get_logger().info(f"💾 Model saved: {saved_path}")

                if loss is not None and self.step_count % 100 == 0:
                    epsilon = self.epsilon_end + (self.epsilon_start - self.epsilon_end) * \
                              np.exp(-1.0 * self.step_count / self.epsilon_decay)
                    fire_or_no = self.current_obs[0]
                    fire_size = self.current_obs[1]
                    angle = self.current_obs[3]
                    self.get_logger().info(
                        f"📊 Step: {self.step_count} | Loss: {loss:.4f} | "
                        f"ε: {epsilon:.3f} | Fire: {fire_or_no:.0f} | Size: {fire_size:.2f} | Angle: {angle:.2f}"
                    )

        self.previous_obs = self.current_obs.copy() if self.current_obs is not None else None

    def save_model(self):
        checkpoint = {
            'step': self.step_count,
            'episode': self.episode_count,
            'model_state_dict': self.agent.policy_net.state_dict(),
            'target_state_dict': self.agent.target_net.state_dict(),
            'optimizer_state_dict': self.agent.optimizer.state_dict(),
        }
        torch.save(checkpoint, self.model_path)
        return self.model_path

    def fire_position_callback(self, msg: PointStamped):
        try:
            self.current_fire_world = (msg.point.x, msg.point.y)
        except Exception:
            self.current_fire_world = None

    def suppressed_fire_callback(self, msg: PointStamped):
        try:
            pos = (msg.point.x, msg.point.y)
            if pos not in self.suppressed_fire_positions:
                self.suppressed_fire_positions.append(pos)
                self.get_logger().info(f"📌 Recorded suppressed fire at {pos}")
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = DQNAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.mode != 'collect':
            final_path = node.save_model()
            node.get_logger().info(f"Interrupted. Final model saved to {final_path}")
        else:
            node.agent.memory.save('replay_buffer.pkl')
            node.get_logger().info("Interrupted. Replay buffer saved to replay_buffer.pkl")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()