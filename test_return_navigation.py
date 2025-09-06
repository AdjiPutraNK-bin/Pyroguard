#!/usr/bin/env python3
"""
Test script for return-to-base navigation improvements
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys
import time

class ReturnToBaseTester(Node):
    def __init__(self):
        super().__init__('return_to_base_tester')
        self.client = self.create_client(Trigger, 'get_fire_detection_info')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for fire detection info service...')
            if not rclpy.ok():
                self.get_logger().error('Service not available')
                sys.exit(1)

    def monitor_navigation(self, duration=60):
        """Monitor return-to-base navigation for a specified duration"""
        self.get_logger().info(f"Monitoring return-to-base navigation for {duration} seconds...")
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.get_fire_info()
            time.sleep(3)  # Check every 3 seconds
            
        self.get_logger().info("Navigation monitoring complete")

    def get_fire_info(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                print("🔥 CURRENT STATUS:")
                print(response.message)
                print("-" * 60)
            else:
                print("❌ Failed to get fire detection info")
        else:
            print("❌ Service call failed")

def main(args=None):
    rclpy.init(args=args)
    tester = ReturnToBaseTester()
    
    print("🧪 Testing Improved Return-to-Base Navigation")
    print("This script will monitor the robot's navigation to base")
    print("Make sure the robot is running and will trigger return-to-base")
    print()
    
    tester.monitor_navigation(60)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
