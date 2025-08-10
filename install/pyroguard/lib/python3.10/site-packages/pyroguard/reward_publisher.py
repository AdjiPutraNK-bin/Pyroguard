#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Bool
import numpy as np

class RewardPublisherNode(Node):
    def __init__(self):
        super().__init__('reward_publisher_node')
        
        # Parameters
        self.declare_parameter('max_steps_per_episode', 1000)
        self.declare_parameter('exploration_reward', 0.05)
        self.max_steps = self.get_parameter('max_steps_per_episode').value
        self.exploration_reward = self.get_parameter('exploration_reward').value

        # Subscribers
        self.obs_sub = self.create_subscription(
            Float32MultiArray, '/obs', self.obs_callback, 10)
        self.vla_trigger_sub = self.create_subscription(
            Bool, '/trigger_vla', self.vla_trigger_callback, 10)
        
        # Publishers
        self.reward_pub = self.create_publisher(Float32, '/reward', 10)
        self.done_pub = self.create_publisher(Bool, '/done_flag', 10)

        # State
        self.current_obs = None
        self.previous_obs = None
        self.step_count = 0
        self.vla_triggered = False

        # Timer for reward computation
        self.timer = self.create_timer(0.1, self.compute_reward)
        
        self.get_logger().info("🚀 Reward Publisher Node initialized")

    def obs_callback(self, msg):
        if len(msg.data) == 3:
            self.current_obs = np.array(msg.data, dtype=np.float32)
        else:
            self.get_logger().error("Invalid observation size: expected 3")

    def vla_trigger_callback(self, msg):
        self.vla_triggered = msg.data

    def compute_reward(self):
        if self.current_obs is None or self.previous_obs is None:
            self.previous_obs = self.current_obs
            return

        distance_to_fire = self.current_obs[0]
        fire_size = self.current_obs[1]
        prev_distance = self.previous_obs[0]
        min_obstacle_distance = self.current_obs[2]

        # Reward calculation
        reward = 0.0
        done = False

        # Fire-seeking rewards
        if distance_to_fire < prev_distance:
            reward += 1.0  # Moving closer to fire
        if self.vla_triggered:
            reward += 10.0  # Fire suppression
            done = True  # Episode done after suppression/removal

        # Obstacle avoidance rewards
        if min_obstacle_distance > 1.0:
            reward += 0.1  # Safe distance
        elif min_obstacle_distance < 0.5:
            reward -= 0.5  # Too close
        if min_obstacle_distance < 0.05:
            reward -= 10.0  # Collision
            done = True

        # Exploration when no fire
        if fire_size < 0.01 and min_obstacle_distance > 1.0:
            reward += self.exploration_reward  # Encourage moving in open areas

        # Timeout
        self.step_count += 1
        if self.step_count >= self.max_steps:
            done = True

        # Publish reward and done flag
        self.reward_pub.publish(Float32(data=reward))
        self.done_pub.publish(Bool(data=done))

        # Reset for next episode
        if done:
            self.step_count = 0
            self.vla_triggered = False
            self.previous_obs = None
        else:
            self.previous_obs = self.current_obs.copy()

def main(args=None):
    rclpy.init(args=args)
    node = RewardPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()