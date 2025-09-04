#!/usr/bin/env python3
"""
Test script for mission complete functionality
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys
import time

class MissionCompleteTester(Node):
    def __init__(self):
        super().__init__('mission_complete_tester')
        self.client = self.create_client(Trigger, 'get_fire_detection_info')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for fire detection info service...')
            if not rclpy.ok():
                self.get_logger().error('Service not available')
                sys.exit(1)

    def monitor_mission_status(self, duration=10):
        """Monitor mission status for a specified duration"""
        self.get_logger().info(f"Monitoring mission status for {duration} seconds...")
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.get_fire_info()
            time.sleep(2)  # Check every 2 seconds
            
        self.get_logger().info("Monitoring complete")

    def get_fire_info(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                print("🔥 CURRENT STATUS:")
                print(response.message)
                print("-" * 50)
            else:
                print("❌ Failed to get fire detection info")
        else:
            print("❌ Service call failed")

def main(args=None):
    rclpy.init(args=args)
    tester = MissionCompleteTester()
    
    print("🧪 Testing Mission Complete Functionality")
    print("This script will monitor the mission status")
    print("If the robot has reached base, Mission Complete should be True")
    print()
    
    tester.monitor_mission_status(10)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
