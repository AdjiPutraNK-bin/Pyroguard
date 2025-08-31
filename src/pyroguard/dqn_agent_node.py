#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import torch
import numpy as np
import os
from std_msgs.msg import Float32MultiArray, Float32, Bool
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

        self.fire_detected_buffer = deque([False]*5, maxlen=5)
        self.avoid_kp = 0.8
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
        self.robot_pose = None

        self.recent_actions = deque(maxlen=10)
        self.action_counts = {i: 0 for i in range(self.action_size)}
        self.stuck_counter = 0

        self.current_target_fire = None
        self.target_lock_start_time = None
        self.current_fire_world = None

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
            self.get_logger().warning("Waiting for required topics: obs=%s, lidar=%s, odom=%s" % 
                                     (self.obs_received, self.lidar_received, self.odom_received))

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
            self.get_logger().warning("No LIDAR or odometry data, stopping robot")
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
            self.get_logger().debug("No obstacles in window, moving forward")
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
            self.get_logger().warning("No valid paths found, moving backward")
            return cmd

        best_window_idx = np.argmin(costs)
        best_angle = window_centers[best_window_idx]
        min_dist = np.min(ranges[np.abs(angles - best_angle) <= window_half / num_windows])

        threshold = self.emergency_threshold * (self.approach_escape_scale if self.approach_mode else 1.0)
        if min_dist < threshold:
            self.get_logger().warn(f"[EMERGENCY] Close to obstacle ({min_dist:.2f}m) - initiating escape")
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
        self.get_logger().info(f"[AVOID] Selected path at angle {best_angle:.2f}, linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f}")
        return cmd

    def execute_escape(self):
        cmd = Twist()
        now = self.get_clock().now().nanoseconds / 1e9
        if self.escape_state == 'backup' and now < self.escape_end_time:
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
        if self.current_obs is None:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().warning("No observation, stopping robot")
            return None, "stop", cmd

        min_obstacle_distance = self.current_obs[2]
        fire_detected = self.current_obs[0] > 0.5
        angle_to_fire = self.current_obs[3]
        fire_distance = self.current_obs[4]

        epsilon = self.epsilon_end + (self.epsilon_start - self.epsilon_end) * \
                  np.exp(-1.0 * self.step_count / self.epsilon_decay)

        if min_obstacle_distance < self.min_safe_distance:
            cmd = self.get_avoidance_cmd()
            if self.approach_mode:
                self.post_avoid_recenter = True
            return None, "avoidance", cmd
        elif self.suppression_mode:
            action = 3
        elif self.approach_mode:
            action = self.navigation_action()
        elif fire_detected:
            action = self.navigation_action()
        else:
            action = self.select_smart_action(epsilon)

        if action is not None:
            action_name = self.action_map[action]
            self.recent_actions.append(action)
            self.action_counts[action] = self.action_counts.get(action, 0) + 1
            cmd = self.execute_action(action_name, action)
        return action, action_name if action is not None else "avoidance", cmd

    def navigation_action(self):
        angle_to_fire = self.current_obs[3]
        fire_distance = self.current_obs[4]
        if abs(angle_to_fire) > self.angle_threshold:
            if angle_to_fire > 0:
                return 2  # turn_right
            else:
                return 1  # turn_left
        else:
            return 5 if fire_distance < self.min_safe_distance * 2.0 else 0  # slow_forward or move_forward

    def select_smart_action(self, epsilon):
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
        cmd = Twist()
        if action_name == "move_forward":
            obstacle_dist = self.current_obs[2] if self.current_obs is not None else self.min_safe_distance
            base_speed = min(0.8, max(0.15, obstacle_dist * 0.3))
            jitter = float(np.random.uniform(-0.05, 0.05))
            speed = max(0.0, min(0.8, base_speed + jitter))
            ang_jitter = float(np.random.uniform(-0.15, 0.15))
            cmd.linear.x = speed
            cmd.angular.z = ang_jitter
        elif action_name == "turn_left":
            cmd.linear.x = 0.05
            cmd.angular.z = self.turn_speed
        elif action_name == "turn_right":
            cmd.linear.x = 0.05
            cmd.angular.z = -self.turn_speed
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

    def control_loop(self):
        if not self.is_ready:
            self.get_logger().warning("Node not ready, skipping control loop")
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        if self.current_obs is None:
            self.get_logger().warning("No observation received, skipping control loop")
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

        fire_detected = self.current_obs[0] > 0.5
        self.fire_detected_buffer.append(fire_detected)
        stable_fire_detected = sum(self.fire_detected_buffer) >= 3

        fire_distance = self.current_obs[4] if len(self.current_obs) >= 5 else 999.0
        current_fire_position = tuple(self.current_obs[1:3]) if len(self.current_obs) >= 3 else None

        current_time = self.get_clock().now().nanoseconds / 1e9
        if (stable_fire_detected and not self.suppression_mode and not self.approach_mode and
            current_fire_position and current_fire_position not in self.suppressed_fire_positions):
            if self.current_target_fire is None:
                self.current_target_fire = current_fire_position
                self.target_lock_start_time = current_time
                self.get_logger().info(f"🔒 Locked onto new target fire at {self.current_target_fire}")
            else:
                self.get_logger().debug(
                    f"Detected fire at {current_fire_position} but focused on {self.current_target_fire}; ignoring"
                )

        if (self.current_target_fire is not None and current_fire_position is not None and
            math.dist(self.current_target_fire, current_fire_position) > 0.5):
            stable_fire_detected = False

        def is_suppressed(pos):
            for s in self.suppressed_fire_positions:
                if s and pos and math.dist(s, pos) < 0.2:
                    return True
            return False
        if current_fire_position and is_suppressed(current_fire_position):
            stable_fire_detected = False

        if self.approach_mode and fire_distance < self.suppression_distance:
            if self.last_recenter_distance is None:
                self.last_recenter_distance = fire_distance
                self.recenter_checkpoints = [fire_distance * (2/3), fire_distance * (1/3)]
            if self.recenter_checkpoints and fire_distance <= self.recenter_checkpoints[0]:
                self.get_logger().info(f"🔄 Recentering to fire at distance {fire_distance:.2f}")
                angle_to_fire = self.current_obs[3]
                if abs(angle_to_fire) > self.angle_threshold:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = (self.turn_speed * 0.5) if angle_to_fire > 0 else -(self.turn_speed * 0.5)
                    self.cmd_pub.publish(twist)
                    return
                self.recenter_checkpoints.pop(0)
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
        elif self.approach_mode:
            if stable_fire_detected and fire_distance <= self.suppression_distance:
                self.get_logger().info("🟢 Fire suppressed, turning away")
                self.escape_state = 'rotate'
                self.escape_end_time = self.get_clock().now().nanoseconds / 1e9 + 3.0
                self.escape_rotation_side = -self.turn_speed
                self.vla_pub.publish(Bool(data=True))
                self.detected_fire_positions = [pos for pos in self.detected_fire_positions if pos != current_fire_position]
                self.approach_mode = False
                self.suppression_mode = False
                self.suppression_mode_pub.publish(Bool(data=False))
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = self.escape_rotation_side
                self.cmd_pub.publish(cmd)
                return
            elif not stable_fire_detected:
                now = self.get_clock().now().nanoseconds / 1e9
                if self.approach_lost_since is None:
                    self.approach_lost_since = now
                    self.get_logger().debug(f"Approach detection lost, starting grace timer")
                elif now - self.approach_lost_since < self.approach_interrupt_timeout:
                    self.get_logger().debug(f"Approach within grace period: {now - self.approach_lost_since:.2f}s")
                else:
                    self.get_logger().info("🟢 Fire lost during approach, exiting approach mode")
                    self.approach_mode = False
                    self.suppression_mode = False
                    self.suppression_mode_pub.publish(Bool(data=False))
                    self.vla_pub.publish(Bool(data=False))
                    self.approach_lost_since = None
            else:
                self.approach_lost_since = None
        elif self.suppression_mode:
            if not stable_fire_detected or fire_distance > self.suppression_distance:
                self.get_logger().info("🟢 Fire lost or too far, exiting suppression mode")
                self.suppression_mode = False
                self.suppression_mode_pub.publish(Bool(data=False))
                self.vla_pub.publish(Bool(data=False))

        action, action_name, cmd = self.select_rl_action()
        if self.post_avoid_recenter and (self.lidar_ranges is None or float(np.min(self.lidar_ranges)) >= self.min_safe_distance):
            angle_to_fire = self.current_obs[3] if (self.current_obs is not None and len(self.current_obs) >= 4) else 0.0
            if abs(angle_to_fire) > self.angle_threshold:
                rec_cmd = Twist()
                rec_cmd.linear.x = 0.0
                rec_cmd.angular.z = self.turn_speed if angle_to_fire > 0 else -self.turn_speed
                self.cmd_pub.publish(rec_cmd)
                self.get_logger().info(f"↪ Recentering toward fire: angle={angle_to_fire:.2f}")
                return
            else:
                self.post_avoid_recenter = False

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f"🤖 Action: {action_name} | linear.x: {cmd.linear.x:.2f} | angular.z: {cmd.angular.z:.2f}"
        )

        if self.previous_obs is not None:
            adjusted_reward = self.reward
            if self.detect_circular_movement():
                adjusted_reward += self.action_repeat_penalty
                self.stuck_counter += 1
                if self.stuck_counter % 10 == 0:
                    self.get_logger().warn("🔄 Circular movement detected - applying penalty")
            else:
                self.stuck_counter = max(0, self.stuck_counter - 1)

            if self.mode != 'inference':
                if action is not None and 0 <= action < self.action_size:
                    self.agent.memory.push(self.previous_obs, action, adjusted_reward, self.current_obs, self.done)
                else:
                    self.get_logger().debug(f"Invalid action index: {action}, skipping memory push.")

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
