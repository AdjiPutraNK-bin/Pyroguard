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

class DQNAgentNode(Node):
    def __init__(self):
        super().__init__('dqn_agent_node')
        self.suppression_mode_pub = self.create_publisher(Bool, '/suppression_mode', 10)
        
        # Debounce buffer for fire detection
        self.fire_detected_buffer = deque([False]*5, maxlen=5)

        # Parameters
        self.declare_parameter('obs_size', 4)  # [fire_or_no, fire_size, lidar_min_distance, angle_to_fire]
        self.declare_parameter('action_size', 4)
        self.declare_parameter('model_path', 'dqn_model.pth')
        self.declare_parameter('epsilon_start', 0.9)
        self.declare_parameter('epsilon_end', 0.05)
        self.declare_parameter('epsilon_decay', 1000)
        self.declare_parameter('save_interval', 1000)
        self.declare_parameter('min_safe_distance', 0.3)
        self.declare_parameter('action_repeat_penalty', -0.5)
        self.declare_parameter('mode', 'train_online')  # 'collect', 'train_online', 'inference'
        self.declare_parameter('lidar_topic', '/world/forest_world/model/turtlebot4/link/lidar_link/sensor/lidar/scan')

        obs_size = self.get_parameter('obs_size').value
        action_size = self.get_parameter('action_size').value
        self.model_path = self.get_parameter('model_path').value
        self.epsilon_start = self.get_parameter('epsilon_start').value
        self.epsilon_end = self.get_parameter('epsilon_end').value
        self.epsilon_decay = self.get_parameter('epsilon_decay').value
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.action_repeat_penalty = self.get_parameter('action_repeat_penalty').value
        self.mode = self.get_parameter('mode').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        if self.mode not in ['collect', 'train_online', 'inference']:
            raise ValueError("Invalid mode: choose 'collect', 'train_online', or 'inference'")

        # Initialize agent
        self.agent = DQNAgent(obs_size, action_size)
        
        # Load pre-trained model if available (for train_online or inference)
        if self.mode != 'collect':
            try:
                if os.path.exists(self.model_path):
                    self.agent.policy_net.load_state_dict(
                        torch.load(self.model_path, map_location=torch.device('cpu')))
                    self.get_logger().info("DQN model loaded successfully")
                else:
                    self.get_logger().warning("No model found - using random weights")
            except (FileNotFoundError, RuntimeError) as e:
                self.get_logger().error(f"Model loading failed: {str(e)} - continuing with random weights")

        # ROS communication
        self.obs_sub = self.create_subscription(
            Float32MultiArray, '/obs', self.obs_callback, 10)
        self.reward_sub = self.create_subscription(
            Float32, '/reward', self.reward_callback, 10)
        self.done_sub = self.create_subscription(
            Bool, '/done_flag', self.done_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, self.lidar_topic, self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vla_pub = self.create_publisher(Bool, '/trigger_vla', 10)

        self.current_obs = None
        self.previous_obs = None
        self.reward = 0.0
        self.done = False
        self.step_count = 0
        self.episode_count = 0
        self.lidar_ranges = None
        self.lidar_angle_increment = None
        self.suppression_mode = False  # Locking mechanism for fire suppression
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz

        # Anti-circular movement measures
        self.recent_actions = deque(maxlen=10)  # Track last 10 actions
        self.action_counts = {i: 0 for i in range(action_size)}
        self.stuck_counter = 0

        # Action map
        self.action_map = {
            0: "move_forward",
            1: "turn_left", 
            2: "turn_right",
            3: "stop"
        }
        self.get_logger().info(f"🤖 Improved DQN Agent Node initialized in mode: {self.mode}")

    def obs_callback(self, msg):
        if len(msg.data) == 8:
            self.current_obs = np.array(msg.data, dtype=np.float32)
            # fire_pose = self.current_obs[5:8] # (fire_x, fire_y, fire_z) available for future use
        else:
            self.get_logger().error(f"Invalid observation size: expected 8, got {len(msg.data)}")
            self.current_obs = None

    def reward_callback(self, msg):
        self.reward = msg.data

    def done_callback(self, msg):
        self.done = msg.data
        if self.done:
            self.episode_count += 1
            self.get_logger().info(
                f"🏁 Episode {self.episode_count} ended. "
                f"Steps: {self.step_count}, Final reward: {self.reward:.2f}"
            )
            self.previous_obs = None
            self.reward = 0.0
            self.done = False
            self.recent_actions.clear()
            self.action_counts = {i: 0 for i in range(5)}
            self.stuck_counter = 0
            self.suppression_mode = False  # Unlock after suppression

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array(msg.ranges, dtype=np.float32)
        self.lidar_angle_increment = msg.angle_increment
        self.lidar_angle_min = msg.angle_min

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

    def get_avoidance_action(self):
        if self.lidar_ranges is None or self.current_obs is None:
            return 3  # Stop if no LIDAR data
        min_distance = self.current_obs[2]  # lidar_min_distance
        angle_to_fire = self.current_obs[3]
        
        # Find angle of closest obstacle
        valid_ranges = self.lidar_ranges[np.isfinite(self.lidar_ranges)]
        if len(valid_ranges) == 0:
            return 0  # Move forward if no valid ranges
        min_range_idx = np.argmin(self.lidar_ranges)
        obstacle_angle = self.lidar_angle_min + min_range_idx * self.lidar_angle_increment
        
        # Divide LIDAR into left and right sectors
        angles = np.arange(self.lidar_angle_min, self.lidar_angle_min + len(self.lidar_ranges) * self.lidar_angle_increment, self.lidar_angle_increment)
        left_sector = self.lidar_ranges[angles < 0]
        right_sector = self.lidar_ranges[angles > 0]
        left_free = np.mean(left_sector[np.isfinite(left_sector)]) if len(left_sector) > 0 else 0.0
        right_free = np.mean(right_sector[np.isfinite(right_sector)]) if len(right_sector) > 0 else 0.0

        # Choose turn direction: prefer side with more free space, bias toward fire
        if left_free > right_free:
            avoidance_action = 1  # Turn left
        else:
            avoidance_action = 2  # Turn right
            
        # Bias toward fire direction if safe
        if abs(angle_to_fire) < np.pi / 2:
            if angle_to_fire < 0 and right_free > self.min_safe_distance:
                avoidance_action = 2  # Turn right toward fire
            elif angle_to_fire > 0 and left_free > self.min_safe_distance:
                avoidance_action = 1  # Turn left toward fire

        self.get_logger().info(f"⚠️ Obstacle at {min_distance:.2f}m, angle {obstacle_angle:.2f} rad, choosing action: {self.action_map[avoidance_action]}")
        return avoidance_action

    def control_loop(self):
        if self.current_obs is None:
            return

        min_obstacle_distance = self.current_obs[2]
        fire_detected = self.current_obs[0] > 0.5
        # Update debounce buffer
        self.fire_detected_buffer.append(fire_detected)
        stable_fire_detected = sum(self.fire_detected_buffer) >= 3  # Require 3/5 detections
        angle_to_fire = self.current_obs[3]

        # Always define epsilon for logging
        epsilon = None

        # Enter suppression mode when fire is stably detected, exit when not
        if stable_fire_detected:
            if not self.suppression_mode:
                self.get_logger().info("🔥 Stable fire detected, entering suppression mode.")
            self.suppression_mode = True
            self.suppression_mode_pub.publish(Bool(data=True))
        else:
            if self.suppression_mode:
                self.get_logger().info("🟢 Fire no longer stably detected, resuming RL control.")
            self.suppression_mode = False
            self.suppression_mode_pub.publish(Bool(data=False))

        if self.suppression_mode:
            # PID-like smooth alignment to fire using bbox_x
            Kp = 1.5  # proportional gain for camera alignment
            cmd = Twist()
            bbox_x = self.current_obs[4] if len(self.current_obs) > 4 else 0.5
            error = bbox_x - 0.5
            # If well aligned, move forward
            if abs(error) < 0.05:
                cmd.linear.x = 0.25
                cmd.angular.z = 0.0
                action = 0
                action_name = self.action_map[action]
            else:
                cmd.linear.x = 0.10
                # Invert sign so left in image means positive angular.z (turn left)
                cmd.angular.z = float(np.clip(-Kp * error, -1.0, 1.0))
                # For logging, show direction
                if error > 0:
                    action = 1  # turn_left
                else:
                    action = 2  # turn_right
                action_name = self.action_map[action]
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"🔒 Suppression mode: {action_name} | bbox_x: {bbox_x:.2f} | error: {error:.2f} | angular.z: {cmd.angular.z:.2f}")
        else:
            if min_obstacle_distance < self.min_safe_distance:
                action = self.get_avoidance_action()
                action_name = self.action_map[action]
                self.get_logger().info(f"⚠️ Applying obstacle avoidance: {action_name}")
            else:
                if self.mode == 'collect':
                    epsilon = 1.0  # Pure random for collection
                elif self.mode == 'inference':
                    epsilon = 0.0  # Greedy
                else:  # train_online
                    epsilon = self.epsilon_end + (self.epsilon_start - self.epsilon_end) * \
                              np.exp(-1.0 * self.step_count / self.epsilon_decay)
                action = self.select_smart_action(epsilon)
                action_name = self.action_map.get(action, "unknown")
                self.execute_action(action_name, action)

            self.recent_actions.append(action)
            self.action_counts[action] += 1

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
                    self.previous_obs, action, adjusted_reward, self.current_obs, self.done
                )
            
            if self.mode == 'train_online':
                loss = self.agent.train_step()
                self.step_count += 1

                if self.step_count % self.get_parameter('save_interval').value == 0:
                    saved_path = self.save_model()
                    self.get_logger().info(f"💾 Model saved: {saved_path}")

                if loss is not None and self.step_count % 100 == 0:
                    fire_or_no = self.current_obs[0]
                    fire_size = self.current_obs[1]
                    angle = self.current_obs[3]
                    if epsilon is not None:
                        self.get_logger().info(
                            f"📊 Step: {self.step_count} | Loss: {loss:.4f} | "
                            f"ε: {epsilon:.3f} | Fire: {fire_or_no:.0f} | Size: {fire_size:.2f} | Angle: {angle:.2f}"
                        )
                    else:
                        self.get_logger().info(
                            f"📊 Step: {self.step_count} | Loss: {loss:.4f} | "
                            f"Fire: {fire_or_no:.0f} | Size: {fire_size:.2f} | Angle: {angle:.2f}"
                        )

        self.previous_obs = self.current_obs.copy() if self.current_obs is not None else None

    def select_smart_action(self, epsilon):
        if np.random.random() < epsilon:
            if self.detect_circular_movement():
                return 0  # move_forward
            else:
                weights = np.ones(4)
                for action, count in self.action_counts.items():
                    if count > 5:
                        weights[action] *= 0.5
                weights[0] *= 2.0
                weights /= weights.sum()
                return np.random.choice(4, p=weights)
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
            cmd.angular.z = 0.6
        elif action_name == "turn_right":
            cmd.linear.x = 0.05
            cmd.angular.z = -0.6
        elif action_name == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def save_model(self):
        model_path = self.model_path
        torch.save({
            'step': self.step_count,
            'episode': self.episode_count,
            'model_state_dict': self.agent.policy_net.state_dict(),
            'target_state_dict': self.agent.target_net.state_dict(),
            'optimizer_state_dict': self.agent.optimizer.state_dict(),
            'epsilon': self.epsilon_end + (self.epsilon_start - self.epsilon_end) * \
                       np.exp(-1.0 * self.step_count / self.epsilon_decay),
        }, model_path)
        return model_path

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