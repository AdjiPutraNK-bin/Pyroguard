#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Bool
import numpy as np
import math

class RewardPublisherNode(Node):
    def __init__(self):
        super().__init__('reward_publisher_node')
        
        # Parameters
        self.declare_parameter('max_steps_per_episode', 1000)
        self.declare_parameter('exploration_reward', 0.01)
        self.declare_parameter('distance_threshold', 0.5)
        self.declare_parameter('step_penalty', -0.01)
        self.declare_parameter('coverage_threshold', 0.95)  # Added: 95% map coverage
        
        self.max_steps = self.get_parameter('max_steps_per_episode').value
        self.exploration_reward = self.get_parameter('exploration_reward').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.step_penalty = self.get_parameter('step_penalty').value
        self.coverage_threshold = self.get_parameter('coverage_threshold').value

        # Subscribers
        self.obs_sub = self.create_subscription(
            Float32MultiArray, '/obs', self.obs_callback, 10)
        self.vla_trigger_sub = self.create_subscription(
            Bool, '/trigger_vla', self.vla_trigger_callback, 10)
        self.all_done_sub = self.create_subscription(
            Bool, '/all_fires_suppressed', self.all_done_callback, 10)
        self.coverage_sub = self.create_subscription(
            Float32, '/map_coverage', self.coverage_callback, 10)
        
        # Publishers
        self.reward_pub = self.create_publisher(Float32, '/reward', 10)
        self.done_pub = self.create_publisher(Bool, '/done_flag', 10)

        # State
        self.current_obs = None
        self.previous_obs = None
        self.step_count = 0
        self.vla_triggered = False
        self.all_suppressed = False
        self.map_coverage = 0.0  # Added: Track map coverage

        # Timer for reward computation
        self.timer = self.create_timer(0.1, self.compute_reward)
        
        self.get_logger().info("🚀 Improved Reward Publisher Node initialized")

    def obs_callback(self, msg):
        if len(msg.data) == 4:
            self.current_obs = np.array(msg.data, dtype=np.float32)
        else:
            self.get_logger().error(f"Invalid observation size: expected 4, got {len(msg.data)}")
            self.current_obs = None

    def vla_trigger_callback(self, msg):
        self.vla_triggered = msg.data

    def all_done_callback(self, msg):
        self.all_suppressed = msg.data

    def coverage_callback(self, msg):
        self.map_coverage = msg.data
        self.get_logger().info(f"Map coverage updated: {self.map_coverage:.2%}")

    def compute_reward(self):
        if self.current_obs is None:
            return
        if self.previous_obs is None:
            self.previous_obs = self.current_obs.copy() if self.current_obs is not None else None
            return

        fire_or_no = self.current_obs[0]
        fire_size = self.current_obs[1]
        min_obstacle_distance = self.current_obs[2]
        angle_to_fire = self.current_obs[3]
        
        reward = 0.0
        done = False

        # 1. FIRE DETECTION REWARD
        if fire_or_no > 0.5:
            reward += 2.0
        else:
            reward -= 0.5

        # 2. DIRECTIONAL ALIGNMENT REWARD
        if abs(angle_to_fire) < np.pi/6:
            reward += 0.5
        elif abs(angle_to_fire) < np.pi/3:
            reward += 0.2
        else:
            reward -= 0.2

        # 3. FIRE SUPPRESSION REWARD
        if self.vla_triggered and fire_or_no > 0.5 and min_obstacle_distance < self.distance_threshold:
            reward += 20.0
            self.get_logger().info("🔥 Fire successfully suppressed!")
        elif self.vla_triggered:
            reward -= 5.0

        # 4. ALL FIRES SUPPRESSED AND MAP COVERED
        if self.all_suppressed and self.map_coverage >= self.coverage_threshold:
            reward += 50.0
            done = True
            self.get_logger().info("🏆 All fires suppressed and map covered! Episode complete.")

        # 5. OBSTACLE AVOIDANCE
        if min_obstacle_distance > 1.0:
            reward += 0.05
        elif min_obstacle_distance < 0.5:
            reward -= 0.5 * (0.5 - min_obstacle_distance)
        if min_obstacle_distance < 0.1:
            reward -= 15.0
            done = True
            self.get_logger().warn("💥 Collision detected!")
        
        # 6. EFFICIENCY INCENTIVES
        reward += self.step_penalty
        
        # 7. EXPLORATION
        if fire_size < 0.01 and min_obstacle_distance > 0.8:
            reward += self.exploration_reward

        # 8. TIMEOUT CHECK
        self.step_count += 1
        if self.step_count >= self.max_steps:
            reward -= 5.0
            done = True
            self.get_logger().info("Episode timeout")

        # Publish results
        self.reward_pub.publish(Float32(data=reward))
        self.done_pub.publish(Bool(data=done))
        
        if self.step_count % 50 == 0:
            self.get_logger().info(
                f"Step {self.step_count}: Fire={fire_or_no:.0f}, "
                f"Size={fire_size:.2f}, Angle={angle_to_fire:.2f}, Reward={reward:.2f}, "
                f"Coverage={self.map_coverage:.2%}"
            )

        if done:
            self.step_count = 0
            self.vla_triggered = False
            self.all_suppressed = False
            self.previous_obs = None
        else:
            self.previous_obs = self.current_obs.copy() if self.current_obs is not None else None

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