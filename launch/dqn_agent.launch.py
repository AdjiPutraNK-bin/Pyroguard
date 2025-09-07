#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pyroguard',
            executable='dqn_agent_node',
            name='dqn_agent_node',
            output='screen',
            parameters=[
                {'mode': 'inference'},
                {'turn_speed': 0.6},
                {'forward_speed': 0.3},
                {'slow_forward_speed': 0.15},
                {'min_safe_distance': 0.8},
                {'suppression_distance': 4.0},
                {'return_to_base_timeout': 60.0}
            ]
        )
    ])