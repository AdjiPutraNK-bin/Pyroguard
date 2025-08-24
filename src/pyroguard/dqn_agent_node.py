#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import torch
import numpy as np
import os
from std_msgs.msg import Float32MultiArray, Float32, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from .dqn_agent import DQNAgent
from collections import deque
import math

class DQNAgentNode(Node):
    def __init__(self):
        super().__init__('dqn_agent_node')
        self.suppression_mode_pub = self.create_publisher(Bool, '/suppression_mode', 10)
        self.vla_pub = self.create_publisher(Bool, '/trigger_vla', 10)

        # Debounce buffer for fire detection
        self.fire_detected_buffer = deque([False]*5, maxlen=5)

        # Avoidance PID parameters
        self.avoid_kp = 0.8  # Proportional gain for avoidance
        self.avoid_kd = 0.2  # Derivative gain for avoidance
        self.avoid_last_error = 0.0

        # Parameters
        self.declare_parameter('obs_size', 5)  # [fire_or_no, fire_size, lidar_min_distance, angle_to_fire, fire_distance]
        self.declare_parameter('action_size', 4)  # [move_forward, turn_left, turn_right, stop]
        self.declare_parameter('model_path', 'dqn_model.pth')
        self.declare_parameter('epsilon_start', 0.9)
        self.declare_parameter('epsilon_end', 0.05)
        self.declare_parameter('epsilon_decay', 1000)
        self.declare_parameter('save_interval', 1000)
        self.declare_parameter('min_safe_distance', 0.4)  # Increased for wider arcs
        self.declare_parameter('suppression_distance', 2.0)
        self.declare_parameter('action_repeat_penalty', -0.5)
        self.declare_parameter('mode', 'train_online')
        self.declare_parameter('lidar_topic', '/world/forest_world/model/turtlebot4/link/lidar_link/sensor/lidar/scan')
        self.declare_parameter('angle_threshold', 0.1)
        self.declare_parameter('turn_speed', 0.6)
        self.declare_parameter('forward_speed', 0.3)
        self.declare_parameter('avoidance_arc_margin', 0.8)  # Distance for wide arcs
        self.declare_parameter('avoidance_max_angular', 0.7)  # Max angular velocity for avoidance

        # Retrieve parameters
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
        self.avoidance_arc_margin = self.get_parameter('avoidance_arc_margin').value
        self.avoidance_max_angular = self.get_parameter('avoidance_max_angular').value

        if self.mode not in ['collect', 'train_online', 'inference']:
            raise ValueError("Invalid mode: choose 'collect', 'train_online', or 'inference'")

        # Initialize DQN agent
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

        # ROS communication
        self.obs_sub = self.create_subscription(Float32MultiArray, '/obs', self.obs_callback, 10)
        self.reward_sub = self.create_subscription(Float32, '/reward', self.reward_callback, 10)
        self.done_sub = self.create_subscription(Bool, '/done_flag', self.done_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, self.lidar_topic, self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State variables
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
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz

        # Anti-circular movement measures
        self.recent_actions = deque(maxlen=10)
        self.action_counts = {i: 0 for i in range(self.action_size)}
        self.stuck_counter = 0

        # Action map
        self.action_map = {
            0: "move_forward",
            1: "turn_left",
            2: "turn_right",
            3: "stop"
        }
        self.get_logger().info(f"🤖 DQN Agent Node initialized in mode: {self.mode}")

    def obs_callback(self, msg):
        if len(msg.data) == self.obs_size:
            self.current_obs = np.array(msg.data, dtype=np.float32)
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

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array(msg.ranges, dtype=np.float32)
        self.lidar_angle_increment = msg.angle_increment
        self.lidar_angle_min = msg.angle_min

    def normalize_angle(self, ang):
        return (ang + math.pi) % (2 * math.pi) - math.pi

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
        if self.lidar_ranges is None or self.current_obs is None:
            return cmd

        min_distance = self.current_obs[2]
        angle_to_fire = float(self.current_obs[3])
        fire_detected = self.current_obs[0] > 0.5

        valid_ranges = self.lidar_ranges[np.isfinite(self.lidar_ranges)]
        if len(valid_ranges) == 0:
            cmd.linear.x = self.forward_speed
            return cmd

        angles = np.arange(self.lidar_angle_min, 
                         self.lidar_angle_min + len(self.lidar_ranges) * self.lidar_angle_increment, 
                         self.lidar_angle_increment)

        # Define finer sectors for better obstacle detection
        sector_size = np.pi / 6  # 30 degrees
        sectors = [
            (-np.pi/2, -np.pi/3),  # Left far (-90° to -60°)
            (-np.pi/3, -np.pi/6),  # Left near (-60° to -30°)
            (-np.pi/6, np.pi/6),   # Center (-30° to 30°)
            (np.pi/6, np.pi/3),    # Right near (30° to 60°)
            (np.pi/3, np.pi/2)     # Right far (60° to 90°)
        ]

        sector_dists = []
        for start, end in sectors:
            mask = (angles >= start) & (angles < end) & np.isfinite(self.lidar_ranges)
            dist = np.nanmean(self.lidar_ranges[mask]) if np.any(mask) else np.inf
            sector_dists.append(dist)

        # Compute error: positive means right is clearer
        left_dist = min(sector_dists[0], sector_dists[1])
        right_dist = min(sector_dists[3], sector_dists[4])
        center_dist = sector_dists[2]
        error = right_dist - left_dist

        # PID-like control for smooth turning
        p = self.avoid_kp * error
        d = self.avoid_kd * (error - self.avoid_last_error)
        angular = p + d
        self.avoid_last_error = error

        # Adjust angular velocity based on obstacle proximity
        if min_distance < self.min_safe_distance:
            # Very close: stop and turn sharply
            cmd.linear.x = 0.0
            cmd.angular.z = self.avoidance_max_angular if angular > 0 else -self.avoidance_max_angular
        elif min_distance < self.avoidance_arc_margin:
            # Create wider arc: slower speed, stronger turn
            cmd.linear.x = max(0.1, min(self.forward_speed, min_distance * 0.25))
            angular_magnitude = min(self.avoidance_max_angular, abs(angular) * (self.avoidance_arc_margin - min_distance) / self.avoidance_arc_margin)
            cmd.angular.z = angular_magnitude if angular > 0 else -angular_magnitude
        else:
            # Open space: move forward with gentle correction
            cmd.linear.x = self.forward_speed
            cmd.angular.z = max(-self.avoidance_max_angular, min(self.avoidance_max_angular, angular * 0.5))

        # Bias toward fire if detected
        if fire_detected:
            fire_bias = 0.4 * angle_to_fire
            cmd.angular.z += fire_bias
            cmd.angular.z = max(-self.avoidance_max_angular, min(self.avoidance_max_angular, cmd.angular.z))

        self.get_logger().debug(
            f"[AVOID] min_dist={min_distance:.2f} left={left_dist:.2f} right={right_dist:.2f} "
            f"center={center_dist:.2f} angular={cmd.angular.z:.2f} linear={cmd.linear.x:.2f}"
        )

        return cmd

    def select_rl_action(self):
        cmd = Twist()
        if self.current_obs is None:
            return 3, self.action_map[3], cmd  # Stop

        min_obstacle_distance = self.current_obs[2]
        fire_detected = self.current_obs[0] > 0.5
        angle_to_fire = self.current_obs[3]
        fire_distance = self.current_obs[4]

        epsilon = self.epsilon_end + (self.epsilon_start - self.epsilon_end) * \
                  np.exp(-1.0 * self.step_count / self.epsilon_decay)

        if min_obstacle_distance < self.min_safe_distance:
            cmd = self.get_avoidance_cmd()
            return None, "avoidance", cmd
        elif self.suppression_mode:
            action = 3
        elif self.approach_mode:
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
        if abs(angle_to_fire) > self.angle_threshold:
            if angle_to_fire > 0:
                return 2  # turn right
            else:
                return 1  # turn left
        else:
            return 0  # move forward

    def select_smart_action(self, epsilon):
        if np.random.random() < epsilon:
            if self.detect_circular_movement():
                return 0
            weights = np.ones(self.action_size)
            for action, count in self.action_counts.items():
                if count > 5:
                    weights[action] *= 0.5
            weights[0] *= 2.0
            weights /= weights.sum()
            return np.random.choice(self.action_size, p=weights)
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
            obstacle_dist = self.current_obs[2]
            speed = min(0.8, max(0.15, obstacle_dist * 0.3))
            cmd.linear.x = speed
            cmd.angular.z = 0.0
        elif action_name == "turn_left":
            cmd.linear.x = 0.05
            cmd.angular.z = self.turn_speed
        elif action_name == "turn_right":
            cmd.linear.x = 0.05
            cmd.angular.z = -self.turn_speed
        elif action_name == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        return cmd

    def control_loop(self):
        if self.current_obs is None:
            self.get_logger().warning("No observation received, skipping control loop")
            return

        self.get_logger().info(f"[DEBUG] current_obs: {self.current_obs}")
        self.get_logger().info(f"[DEBUG] fire_detected_buffer: {list(self.fire_detected_buffer)}")

        fire_detected = self.current_obs[0] > 0.5
        self.fire_detected_buffer.append(fire_detected)
        stable_fire_detected = sum(self.fire_detected_buffer) >= 3

        fire_distance = self.current_obs[4] if len(self.current_obs) >= 5 else 999.0

        if stable_fire_detected and not self.suppression_mode and not self.approach_mode:
            self.get_logger().info("🔥 Stable fire detected, entering approach mode")
            self.approach_mode = True
            self.locked_angle = self.current_obs[3]
            self.suppression_mode_pub.publish(Bool(data=False))
            self.vla_pub.publish(Bool(data=False))
        elif self.approach_mode:
            if stable_fire_detected and fire_distance <= self.suppression_distance:
                self.get_logger().info("🟢 Fire suppressed, entering suppression mode")
                self.approach_mode = False
                self.suppression_mode = True
                self.suppression_mode_pub.publish(Bool(data=True))
                self.vla_pub.publish(Bool(data=True))
            elif not stable_fire_detected:
                self.get_logger().info("🟢 Fire lost during approach, exiting approach mode")
                self.approach_mode = False
                self.suppression_mode = False
                self.suppression_mode_pub.publish(Bool(data=False))
                self.vla_pub.publish(Bool(data=False))
        elif self.suppression_mode:
            if not stable_fire_detected or fire_distance > self.suppression_distance:
                self.get_logger().info("🟢 Fire lost or too far, exiting suppression mode")
                self.suppression_mode = False
                self.suppression_mode_pub.publish(Bool(data=False))
                self.vla_pub.publish(Bool(data=False))

        action, action_name, cmd = self.select_rl_action()
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
                self.agent.memory.push(
                    self.previous_obs, action if action is not None else -1, adjusted_reward, self.current_obs, self.done
                )

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