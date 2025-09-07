#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    # Paths and configurations
    package_share_dir = get_package_share_directory('pyroguard')
    world_path = os.path.join(package_share_dir, 'worlds', 'forest.pruned.sdf')
    custom_gazebo_dir = os.path.join(package_share_dir, 'worlds')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model = LaunchConfiguration('model', default='standard')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='3.5')
    yaw = LaunchConfiguration('yaw', default='0.0')
    run_nodes = LaunchConfiguration('run_nodes', default='true')
    run_simulation = LaunchConfiguration('run_simulation', default='true')

    # Environment variables
    ros_share_path = '/opt/ros/humble/share'
    current_ign_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ign_gazebo_resource_path_env = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        f"{custom_gazebo_dir}:{ros_share_path}:{current_ign_path}"
    )
    ign_gazebo_system_plugin_path_env = SetEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        '/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib'
    )
    ld_library_path_env = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        '/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu'
    )
    gz_plugin_path_env = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        '/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib'
    )
    rmw_implementation_env = SetEnvironmentVariable(
        'RMW_IMPLEMENTATION',
        'rmw_fastrtps_cpp'
    )

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_model_cmd = DeclareLaunchArgument(
        'model',
        default_value='standard',
        choices=['standard', 'lite'],
        description='TurtleBot4 model: standard (with OAK-D camera) or lite (no camera)'
    )
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x position of the robot'
    )
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial y position of the robot'
    )
    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='3.5',
        description='Initial z position of the robot'
    )
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw orientation of the robot'
    )
    declare_run_nodes_cmd = DeclareLaunchArgument(
        'run_nodes',
        default_value='true',
        description='Run Pyroguard nodes if true'
    )
    declare_run_simulation_cmd = DeclareLaunchArgument(
        'run_simulation',
        default_value='true',
        description='Run Gazebo simulation if true'
    )

    # Simulation components
    turtlebot4_world_launch = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen',
        condition=LaunchConfigurationEquals('run_simulation', 'true')
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        condition=LaunchConfigurationEquals('run_simulation', 'true')
    )

    service_bridge_node = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['echo', '🔗 DEBUG: Starting ros_gz service bridges (create/remove/set_pose/control)'],
                output='screen'
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='service_bridges',
                arguments=[
                    '/world/forest_world/create@ros_gz_interfaces/srv/SpawnEntity',
                    '/world/forest_world/remove@ros_gz_interfaces/srv/DeleteEntity',
                    '/world/forest_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                    '/world/forest_world/control@ros_gz_interfaces/srv/ControlWorld',
                ],
                output='screen'
            )
        ],
        condition=LaunchConfigurationEquals('run_simulation', 'true')
    )

    pose_and_odom_bridge = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['echo', '🔗 DEBUG: Starting odom + world pose/info bridges (delayed)'],
                output='screen'
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='pose_and_odom_bridge',
                arguments=[
                    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/world/forest_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                ],
                output='screen'
            )
        ],
        condition=LaunchConfigurationEquals('run_simulation', 'true')
    )

    turtlebot4_spawn_launch = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['echo', '🤖 DEBUG: Starting CUSTOM robot spawn at 8 seconds (bypassing built-in spawner)...'],
                output='screen'
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': xacro.process_file('/opt/ros/humble/share/turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro').toxml(),
                    'use_sim_time': use_sim_time,
                }],
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_turtlebot4',
                arguments=[
                    '-name', 'turtlebot4',
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', z_pose,
                    '-Y', yaw,
                    '-topic', 'robot_description'
                ],
                output='screen'
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='essential_cmd_vel_bridge',
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
                ],
                output='screen'
            ),
        ],
        condition=LaunchConfigurationEquals('run_simulation', 'true')
    )

    lidar_bridge_node = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['echo', '🔍 DEBUG: Starting LiDAR bridge at 10 seconds...'],
                output='screen'
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='lidar_bridge',
                arguments=[
                    '/world/forest_world/model/turtlebot4/link/lidar_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
                ],
                output='screen'
            )
        ],
        condition=LaunchConfigurationEquals('run_simulation', 'true')
    )

    camera_bridge_node = TimerAction(
    period=16.0,
    actions=[
        ExecuteProcess(
            cmd=['echo', '📷 DEBUG: Starting camera bridge at 16 seconds...'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge_manual',
            arguments=[
                '/world/forest_world/model/turtlebot4/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '--ros-args', '-r', '/world/forest_world/model/turtlebot4/link/camera_link/sensor/camera/image:=/test_camera'
            ],
            output='screen'
        )
    ],
    condition=LaunchConfigurationEquals('run_simulation', 'true')
)
    

    # Pyroguard nodes
    pyroguard_nodes = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=['echo', '🔥 DEBUG: Starting Pyroguard nodes at 20 seconds...'],
                output='screen'
            ),
            Node(
                package='pyroguard',
                executable='image_preprocessor_node',
                name='image_preprocessor_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            Node(
                package='pyroguard',
                executable='fire_node',
                name='fire_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            Node(
                package='pyroguard',
                executable='lidar_vla_processor_node',
                name='lidar_vla_processor_node',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'min_safe_distance': 0.3}
                ]
            ),
            Node(
                package='pyroguard',
                executable='fire_suppression_handler_node',
                name='fire_suppression_handler_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            Node(
                package='pyroguard',
                executable='reward_node',
                name='reward_node',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'min_safe_distance': 0.3}
                ]
            ),
            Node(
                package='pyroguard',
                executable='map_coverage_node',
                name='map_coverage_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            Node(
                package='pyroguard',
                executable='dqn_agent_node',
                name='dqn_agent_node',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'mode': 'train_online'},
                    {'obs_size': 5},
                    {'action_size': 6},
                    {'model_path': 'dqn_model.pth'},
                    {'epsilon_start': 0.9},
                    {'epsilon_end': 0.05},
                    {'epsilon_decay': 1000},
                    {'save_interval': 1000},
                    {'min_safe_distance': 0.3},
                    {'action_repeat_penalty': -0.5},
                    {'avoidance_timeout': 2.0},
                ]
            ),
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='async_slam_toolbox_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
        ],
        condition=LaunchConfigurationEquals('run_nodes', 'true')
    )

    # Optional test node
    test_node = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=['echo', '🧪 DEBUG: Starting test navigation node at 25 seconds...'],
                output='screen'
            ),
            Node(
                package='pyroguard',
                executable='test_navigation',
                name='test_navigation_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
        ],
        condition=LaunchConfigurationEquals('run_nodes', 'true')
    )

    return LaunchDescription([
        # Environment variables
        ign_gazebo_resource_path_env,
        ign_gazebo_system_plugin_path_env,
        ld_library_path_env,
        gz_plugin_path_env,
        rmw_implementation_env,

        # Launch arguments
        declare_use_sim_time_cmd,
        declare_model_cmd,
        declare_x_position_cmd,
        declare_y_position_cmd,
        declare_z_position_cmd,
        declare_yaw_cmd,
        declare_run_nodes_cmd,
        declare_run_simulation_cmd,

        # Simulation components
        turtlebot4_world_launch,
        clock_bridge,
        service_bridge_node,
        pose_and_odom_bridge,
        turtlebot4_spawn_launch,
        lidar_bridge_node,
        camera_bridge_node,
        

        # Pyroguard nodes
        pyroguard_nodes,

        # Optional test node (uncomment to include)
        #test_node,
    ])