from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='collect',
        description='DQN agent mode: collect, train_online, or inference'
    )
    
    return LaunchDescription([
        mode_arg,
        
        # Image Preprocessor Node
        Node(
            package='pyroguard',
            executable='image_preprocessor.py',
            name='image_preprocessor',
            output='screen'
        ),
        
        # Fire Detection Node (YOLO)
        Node(
            package='pyroguard',
            executable='fire_node.py',
            name='fire_node',
            output='screen',
            parameters=[
                {'model_path': '/home/adji714/ros2_ws/src/pyroguard/models/best.pt'},
                {'confidence_threshold': 0.5}
            ]
        ),
        
        # LIDAR-VLA Processor Node
        Node(
            package='pyroguard',
            executable='lidar_vla_processor.py',
            name='lidar_vla_processor',
            output='screen',
            parameters=[
                {'min_safe_distance': 0.3},
                {'lidar_topic': '/world/forest_world/model/turtlebot4/link/lidar_link/sensor/lidar/scan'},
                {'world_name': 'forest_world'}
            ]
        ),
        
        # Fire Suppression Handler Node
        Node(
            package='pyroguard',
            executable='fire_suppression_handler.py',
            name='fire_suppression_handler',
            output='screen',
            parameters=[
                {'fov_deg': 30.0},
                {'max_range': 6.0},
                {'world_name': 'forest_world'},
                {'debug_detection': True}
            ]
        ),
        
        # Reward Publisher Node
        Node(
            package='pyroguard',
            executable='reward_publisher.py',
            name='reward_publisher',
            output='screen',
            parameters=[
                {'max_steps_per_episode': 1000},
                {'exploration_reward': 0.01},
                {'distance_threshold': 0.5},
                {'step_penalty': -0.01},
                {'coverage_threshold': 0.95}
            ]
        ),
        
        # Map Coverage Node
        Node(
            package='pyroguard',
            executable='map_coverage_node.py',
            name='map_coverage',
            output='screen',
            parameters=[
                {'coverage_threshold': 0.95},
                {'map_topic': '/map'},
                {'world_name': 'forest_world'},
                {'map_save_path': 'fire_map.png'}
            ]
        ),
        
        # DQN Agent Node
        Node(
            package='pyroguard',
            executable='dqn_agent_node.py',
            name='dqn_agent_node',
            output='screen',
            parameters=[
                {'obs_size': 4},
                {'action_size': 5},
                {'model_path': 'dqn_model_offline.pth'},
                {'epsilon_start': 0.9},
                {'epsilon_end': 0.05},
                {'epsilon_decay': 1000},
                {'save_interval': 1000},
                {'min_safe_distance': 0.3},
                {'action_repeat_penalty': -0.5},
                {'mode': LaunchConfiguration('mode')}
            ]
        ),
        
        # SLAM Toolbox Node (online_async for mapping)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'odom_topic': '/odom'},
                {'scan_topic': '/world/forest_world/model/turtlebot4/link/lidar_link/sensor/lidar/scan'},
                {'map_frame': 'map'},
                {'base_frame': 'base_link'},
                {'resolution': 0.05},
                {'max_laser_range': 20.0}
            ]
        )
    ])