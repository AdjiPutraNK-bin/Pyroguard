#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
import subprocess
import sys
import time

class CustomSpawner(Node):
    def __init__(self, controller_name):
        super().__init__('custom_spawner_' + controller_name.replace(" ", "_"))
        self.controller_name = controller_name
        self.cli = self.create_client(ListControllers, '/controller_manager/list_controllers')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service /controller_manager/list_controllers not available, waiting again...')
        self.get_logger().info('Service /controller_manager/list_controllers is available.')
        self.spawn_controller()

    def spawn_controller(self):
        self.get_logger().info(f"Spawning controller: {self.controller_name}")
        try:
            process = subprocess.run(
                ['ros2', 'run', 'controller_manager', 'spawner', self.controller_name],
                check=True,
                capture_output=True,
                text=True
            )
            self.get_logger().info(f"Successfully spawned {self.controller_name}")
            self.get_logger().info(process.stdout)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to spawn {self.controller_name}")
            self.get_logger().error(f"Stderr: {e.stderr}")
            self.get_logger().error(f"Stdout: {e.stdout}")
        finally:
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: custom_spawner <controller_name>")
        return
    controller_name = sys.argv[1]
    spawner = CustomSpawner(controller_name)
    # No need to spin, the node will shutdown on its own.

if __name__ == '__main__':
    main()
