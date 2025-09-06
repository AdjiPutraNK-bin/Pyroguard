#!/usr/bin/env python3
"""
DQN Training Monitor
Monitors training progress and provides real-time feedback
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Bool
import time
import threading
import signal
import sys

class TrainingMonitor(Node):
    def __init__(self):
        super().__init__('training_monitor')

        # Subscribers for monitoring
        self.obs_sub = self.create_subscription(
            Float32MultiArray, '/obs', self.obs_callback, 10)
        self.reward_sub = self.create_subscription(
            Float32, '/reward', self.reward_callback, 10)
        self.done_sub = self.create_subscription(
            Bool, '/done_flag', self.done_callback, 10)

        # Tracking variables
        self.step_count = 0
        self.episode_count = 0
        self.current_reward = 0.0
        self.episode_rewards = []
        self.start_time = time.time()

        # Fire detection tracking
        self.fire_detected_count = 0
        self.last_obs = None

        self.get_logger().info("🔍 Training Monitor Started")

    def obs_callback(self, msg):
        """Monitor observation data"""
        self.last_obs = msg.data
        self.step_count += 1

        # Check for fire detection
        if len(msg.data) > 0 and msg.data[0] > 0.5:
            self.fire_detected_count += 1

        # Log progress every 100 steps
        if self.step_count % 100 == 0:
            elapsed = time.time() - self.start_time
            rate = self.step_count / elapsed if elapsed > 0 else 0

            self.get_logger().info(
                f"📊 Step: {self.step_count} | "
                f"Rate: {rate:.1f} steps/sec | "
                f"Fires: {self.fire_detected_count}"
            )

    def reward_callback(self, msg):
        """Monitor reward signals"""
        self.current_reward = msg.data

    def done_callback(self, msg):
        """Monitor episode completions"""
        if msg.data:
            self.episode_count += 1
            self.episode_rewards.append(self.current_reward)

            # Calculate average reward
            avg_reward = sum(self.episode_rewards[-10:]) / min(10, len(self.episode_rewards))

            self.get_logger().info(
                f"🏁 Episode {self.episode_count} | "
                f"Reward: {self.current_reward:.3f} | "
                f"Avg (last 10): {avg_reward:.3f}"
            )

            self.current_reward = 0.0

    def print_summary(self):
        """Print training summary"""
        elapsed = time.time() - self.start_time
        total_episodes = len(self.episode_rewards)

        if total_episodes > 0:
            avg_reward = sum(self.episode_rewards) / total_episodes
            best_reward = max(self.episode_rewards)
            worst_reward = min(self.episode_rewards)

            print("\n" + "="*50)
            print("📊 TRAINING SUMMARY")
            print("="*50)
            print(f"Total Steps: {self.step_count}")
            print(f"Total Episodes: {total_episodes}")
            print(".1f")
            print(".3f")
            print(".3f")
            print(".3f")
            print(f"Fires Detected: {self.fire_detected_count}")
            print("="*50)

def main():
    # Handle graceful shutdown
    def signal_handler(sig, frame):
        print('\n🛑 Monitor shutting down...')
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    rclpy.init()
    monitor = TrainingMonitor()

    try:
        print("🔍 Starting training monitor...")
        print("Press Ctrl+C to stop monitoring")

        # Run in background thread
        thread = threading.Thread(target=rclpy.spin, args=(monitor,))
        thread.daemon = True
        thread.start()

        # Keep main thread alive
        while rclpy.ok():
            time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        monitor.print_summary()
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
