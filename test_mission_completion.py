#!/usr/bin/env python3
"""
Test script for mission completion logic
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys
import time

class MissionCompletionTester(Node):
    def __init__(self):
        super().__init__('mission_completion_tester')
        self.client = self.create_client(Trigger, 'get_fire_detection_info')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for fire detection info service...')
            if not rclpy.ok():
                self.get_logger().error('Service not available')
                sys.exit(1)

    def test_mission_completion(self):
        """Test mission completion by monitoring fire detection status"""
        self.get_logger().info("Testing mission completion logic...")
        
        # Monitor for 30 seconds to see if mission completes
        for i in range(10):
            self.get_fire_info()
            time.sleep(3)
            
        self.get_logger().info("Mission completion test complete")

    def get_fire_info(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                print("🔥 MISSION STATUS:")
                print(response.message)
                print("-" * 60)
            else:
                print("❌ Failed to get fire detection info")
        else:
            print("❌ Service call failed")

def main(args=None):
    rclpy.init(args=args)
    tester = MissionCompletionTester()
    
    print("🧪 Testing Mission Completion Logic")
    print("This script will monitor the robot's mission status")
    print("Make sure the robot is running and fires are being suppressed")
    print()
    
    tester.test_mission_completion()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
