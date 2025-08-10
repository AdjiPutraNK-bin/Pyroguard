#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import torch
import numpy as np
import os
from std_msgs.msg import Float32MultiArray, Float32, Bool
from geometry_msgs.msg import Twist
from .dqn_agent import DQNAgent

class DQNAgentNode(Node):
    def __init__(self):
        super().__init__('dqn_agent_node')
        
        # Parameters
        self.declare_parameter('obs_size', 3)  # [distance_to_fire, fire_size, min_obstacle_distance]
        self.declare_parameter('action_size', 5)
        self.declare_parameter('model_path', 'dqn_model.pth')
        self.declare_parameter('epsilon_start', 0.9)
        self.declare_parameter('epsilon_end', 0.05)
        self.declare_parameter('epsilon_decay', 1000)
        self.declare_parameter('save_interval', 1000)
        self.declare_parameter('min_safe_distance', 0.3)

        obs_size = self.get_parameter('obs_size').value
        action_size = self.get_parameter('action_size').value
        self.model_path = self.get_parameter('model_path').value
        self.epsilon_start = self.get_parameter('epsilon_start').value
        self.epsilon_end = self.get_parameter('epsilon_end').value
        self.epsilon_decay = self.get_parameter('epsilon_decay').value
        self.min_safe_distance = self.get_parameter('min_safe_distance').value

        # Initialize agent
        self.agent = DQNAgent(obs_size, action_size)
        
        # Load pre-trained model if available
        try:
            if os.path.exists(self.model_path):
                self.agent.policy_net.load_state_dict(
                    torch.load(self.model_path, map_location=torch.device('cpu')))
                self.get_logger().info("✅ DQN model loaded successfully")
            else:
                self.get_logger().warning("⚠️ No model found - using random weights")
        except Exception as e:
            self.get_logger().error(f"Model loading failed: {str(e)}")
            rclpy.shutdown()

        # ROS communication
        self.obs_sub = self.create_subscription(
            Float32MultiArray, '/obs', self.obs_callback, 10)
        self.reward_sub = self.create_subscription(
            Float32, '/reward', self.reward_callback, 10)
        self.done_sub = self.create_subscription(
            Bool, '/done_flag', self.done_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vla_pub = self.create_publisher(Bool, '/trigger_vla', 10)

        self.current_obs = None
        self.previous_obs = None
        self.reward = 0.0
        self.done = False
        self.step_count = 0
        self.episode_count = 0
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz

        # Action map
        self.action_map = {
            0: "move_forward",
            1: "turn_left",
            2: "turn_right",
            3: "stop",
            4: "trigger_vla"
        }
        self.get_logger().info("🚀 DQN Agent Node initialized for fire-seeking with obstacle avoidance")

    def obs_callback(self, msg):
        if len(msg.data) == 3:
            self.current_obs = np.array(msg.data, dtype=np.float32)
        else:
            self.get_logger().error("Invalid observation size: expected 3 (distance, size, obstacle_distance)")

    def reward_callback(self, msg):
        self.reward = msg.data

    def done_callback(self, msg):
        self.done = msg.data
        if self.done:
            self.episode_count += 1
            self.get_logger().info(f"Episode {self.episode_count} ended. Total steps: {self.step_count}")
            self.previous_obs = None
            self.reward = 0.0
            self.done = False

    def control_loop(self):
        if self.current_obs is None:
            return

        # Safety layer: override action if obstacle too close
        min_obstacle_distance = self.current_obs[2]
        action_name = "stop" if min_obstacle_distance < self.min_safe_distance else None

        if action_name is None:
            # Calculate epsilon with decay
            epsilon = self.epsilon_end + (self.epsilon_start - self.epsilon_end) * \
                      np.exp(-1.0 * self.step_count / self.epsilon_decay)
            # Select action
            action = self.agent.select_action(self.current_obs, epsilon)
            action_name = self.action_map.get(action, "unknown")

        self.execute_action(action_name)

        # Online training
        if self.previous_obs is not None:
            self.agent.memory.push(self.previous_obs, action, self.reward, self.current_obs, self.done)
            loss = self.agent.train_step()
            self.step_count += 1

            # Save model periodically
            if self.step_count % self.get_parameter('save_interval').value == 0:
                saved_path = self.save_model()
                self.get_logger().info(f"Saved model to {saved_path}")

            # Log progress
            if loss is not None and self.step_count % 100 == 0:
                self.get_logger().info(
                    f"Step: {self.step_count} | Loss: {loss:.4f} | Epsilon: {epsilon:.3f}"
                )

        self.previous_obs = self.current_obs.copy()

    def execute_action(self, action_name):
        cmd = Twist()
        if action_name == "move_forward":
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
        elif action_name == "turn_left":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
        elif action_name == "turn_right":
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5
        elif action_name == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif action_name == "trigger_vla":
            self.vla_pub.publish(Bool(data=True))
            self.get_logger().info("🔥 Fire suppression activated!")
            return
        self.cmd_pub.publish(cmd)

    def save_model(self):
        model_path = f"{self.model_path}_step_{self.step_count}.pth"
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
        final_path = node.save_model()
        node.get_logger().info(f"Interrupted. Final model saved to {final_path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()