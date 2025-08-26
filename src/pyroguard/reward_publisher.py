#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Bool
from geometry_msgs.msg import PointStamped
import numpy as np
import math

class RewardPublisherNode(Node):
    def __init__(self):
        super().__init__('reward_publisher_node')
        
        # Parameters
        self.declare_parameter('max_steps_per_episode', 1000)
        self.declare_parameter('exploration_reward', 0.01)
        self.declare_parameter('distance_threshold', 1.2)
        self.declare_parameter('suppression_distance', 4.0)
        self.declare_parameter('step_penalty', -0.01)
        self.declare_parameter('coverage_threshold', 0.95)
        self.declare_parameter('obs_size', 5)
        self.declare_parameter('focus_bonus', 0.1)  # Reward for maintaining focus on a single fire
        self.declare_parameter('switch_penalty', -0.3)  # Penalty for switching targets too frequently
        
        self.exploration_reward = self.get_parameter('exploration_reward').value
        self.max_steps = self.get_parameter('max_steps_per_episode').value            
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.suppression_distance = self.get_parameter('suppression_distance').value
        self.step_penalty = self.get_parameter('step_penalty').value
        self.coverage_threshold = self.get_parameter('coverage_threshold').value
        self.obs_size = self.get_parameter('obs_size').value
        self.focus_bonus = self.get_parameter('focus_bonus').value
        self.switch_penalty = self.get_parameter('switch_penalty').value

        # Subscribers
        self.obs_sub = self.create_subscription(
            Float32MultiArray, '/obs', self.obs_callback, 10)
        self.suppression_event_sub = self.create_subscription(
            PointStamped, '/suppressed_fire_position', self.suppression_event_callback, 10)
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
        self.suppression_event = False
        self.all_suppressed = False
        self.map_coverage = 0.0
        self.current_target_fire = None
        self.last_fire_switch_time = 0
        self.time_on_current_target = 0

        # Timer for reward computation
        self.timer = self.create_timer(0.1, self.compute_reward)
        
        self.get_logger().info("🚀 Improved Reward Publisher Node initialized")

    def obs_callback(self, msg):
        if len(msg.data) == self.obs_size:
            self.current_obs = np.array(msg.data, dtype=np.float32)
            
            # Track target fire based on observations
            fire_detected = self.current_obs[0] > 0.5
            if fire_detected:
                current_fire_position = tuple(self.current_obs[1:3]) if len(self.current_obs) >= 3 else None
                
                if self.current_target_fire is None:
                    self.current_target_fire = current_fire_position
                    self.last_fire_switch_time = self.get_clock().now().nanoseconds / 1e9
                elif current_fire_position and math.dist(self.current_target_fire, current_fire_position) > 0.5:
                    # Fire position has changed significantly, might be a new fire
                    time_since_switch = (self.get_clock().now().nanoseconds / 1e9) - self.last_fire_switch_time
                    if time_since_switch > 5.0:  # Only switch after minimum time
                        self.current_target_fire = current_fire_position
                        self.last_fire_switch_time = self.get_clock().now().nanoseconds / 1e9
        else:
            self.get_logger().error(f"Invalid observation size: expected {self.obs_size}, got {len(msg.data)}")
            self.current_obs = None

    def suppression_event_callback(self, msg):
        self.suppression_event = True

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
        fire_distance = self.current_obs[4]
        
        reward = 0.0
        done = False

        # 1. FIRE DETECTION REWARD
        if fire_or_no > 0.5:
            # Proximity bonus: if robot is close to fire, give extra reward
            if fire_distance < self.suppression_distance:
                reward += 5.0
                self.get_logger().info(f"🔥 Fire detected and within {self.suppression_distance}m! Proximity bonus applied.")
            else:
                reward += 2.0
        else:
            reward -= 0.5

        # 2. DIRECTIONAL ALIGNMENT REWARD
        # Only reward alignment if fire is detected
        if fire_or_no > 0.5:
            if abs(angle_to_fire) < np.pi/12:
                reward += 1.0
                self.get_logger().info("🚩 Robot well aligned with fire!")
            elif abs(angle_to_fire) < np.pi/6:
                reward += 0.5
            elif abs(angle_to_fire) < np.pi/3:
                reward += 0.2
            else:
                reward -= 0.2

        # 3. FIRE SUPPRESSION REWARD
        if self.suppression_event and fire_or_no > 0.5 and fire_distance < self.suppression_distance:
            reward += 20.0
            self.get_logger().info(f"🔥 Fire successfully suppressed within {self.suppression_distance}m!")
            self.suppression_event = False

        # 4. FOCUS REWARD - Encourage maintaining focus on a single fire
        current_time = self.get_clock().now().nanoseconds / 1e9
        time_on_target = current_time - self.last_fire_switch_time
        
        if time_on_target > 5.0:  # After 5 seconds on the same target
            reward += self.focus_bonus * (time_on_target / 5.0)  # Scale with time on target
            
        # 5. SWITCHING PENALTY - Discourage frequent switching
        if time_on_target < 3.0 and fire_or_no > 0.5:
            reward += self.switch_penalty * (3.0 - time_on_target) / 3.0

        # 6. ALL FIRES SUPPRESSED AND MAP COVERED
        if self.all_suppressed and self.map_coverage >= self.coverage_threshold:
            reward += 50.0
            done = True
            self.get_logger().info("🏆 All fires suppressed and map covered! Episode complete.")

        # 7. OBSTACLE AVOIDANCE
        if min_obstacle_distance > 1.0:
            reward += 0.05
        elif min_obstacle_distance < 0.5:
            reward -= 0.5 * (0.5 - min_obstacle_distance)
        if min_obstacle_distance < 0.1:
            reward -= 15.0
            done = True
            self.get_logger().warn("💥 Collision detected!")
        
        # 8. EFFICIENCY INCENTIVES
        reward += self.step_penalty
        
        # 9. EXPLORATION
        if fire_size < 0.01 and min_obstacle_distance > 0.8:
            reward += self.exploration_reward

        # Removed: Timeout check
        self.step_count += 1

        # Publish results
        self.reward_pub.publish(Float32(data=reward))
        self.done_pub.publish(Bool(data=done))
        
        if self.step_count % 50 == 0:
            self.get_logger().info(
                f"Step {self.step_count}: Fire={fire_or_no:.0f}, "
                f"Size={fire_size:.2f}, Angle={angle_to_fire:.2f}, Reward={reward:.2f}, "
                f"Coverage={self.map_coverage:.2%}, Time on target={time_on_target:.1f}s"
            )

        if done:
            self.step_count = 0
            self.suppression_event = False
            self.all_suppressed = False
            self.previous_obs = None
            self.current_target_fire = None
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