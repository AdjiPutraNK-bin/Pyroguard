from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='pyroguard', executable='image_preprocessor_node', name='image_preprocessor_node'),
        Node(package='pyroguard', executable='fire_segmentation_node', name='fire_segmentation_node'),
        Node(package='pyroguard', executable='lidar_vla_processor_node', name='lidar_vla_processor_node'),
        Node(package='pyroguard', executable='reward_publisher_node', name='reward_publisher_node'),
        Node(package='pyroguard', executable='fire_suppression_handler_node', name='fire_suppression_handler_node'),
        Node(package='pyroguard', executable='dqn_agent_node', name='dqn_agent_node'),
    ])