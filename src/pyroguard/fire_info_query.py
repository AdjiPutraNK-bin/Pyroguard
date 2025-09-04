#!/usr/bin/env python3
"""
Fire Detection Info Query Script
Call this script to get detailed information about fire detection timing
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys

class FireInfoClient(Node):
    def __init__(self):
        super().__init__('fire_info_client')
        self.client = self.create_client(Trigger, 'get_fire_detection_info')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for fire detection info service...')
            if not rclpy.ok():
                self.get_logger().error('Service not available')
                sys.exit(1)

    def get_fire_info(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                print("🔥 FIRE DETECTION STATUS:")
                print(response.message)
            else:
                print("❌ Failed to get fire detection info")
        else:
            print("❌ Service call failed")

def main(args=None):
    rclpy.init(args=args)
    client = FireInfoClient()
    client.get_fire_info()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
