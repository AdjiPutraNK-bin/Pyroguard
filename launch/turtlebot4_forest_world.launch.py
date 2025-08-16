#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Paths and configurations
    package_share_dir = get_package_share_directory('pyroguard')
    world_path = os.path.join(package_share_dir, 'worlds', 'forest.sdf')
    custom_gazebo_world_path = world_path
    custom_gazebo_dir = os.path.join(package_share_dir, 'worlds')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model = LaunchConfiguration('model', default='standard')  # Force standard model with camera
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='3.5')
    yaw = LaunchConfiguration('yaw', default='0.0')
    
    # Environment variables for plugin compatibility
    ros_share_path = '/opt/ros/humble/share'
    current_ign_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    # Only use local package worlds dir for resources
    ign_gazebo_resource_path_env = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        f"{custom_gazebo_dir}:{ros_share_path}:{current_ign_path}"
    )
    
    # Set correct plugin paths for ros_gz_bridge compatibility
    ign_gazebo_system_plugin_path_env = SetEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        '/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib'
    )
    
    ldn_library_path_env = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        '/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu'
    )
    
    gz_plugin_path_env = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        '/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib'
    )
    
    # Force use of FastRTPS to avoid SHM errors
    rmw_implementation_env = SetEnvironmentVariable(
        'RMW_IMPLEMENTATION',
        'rmw_fastrtps_cpp'
    )
    
    # Launch arguments declarations
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
    
    # Use local worlds/forest.sdf for the simulation world
    local_world_path = os.path.join(package_share_dir, 'worlds', 'forest.sdf')
    turtlebot4_world_launch = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', local_world_path],
        output='screen'
    )
    
    # Clock bridge for sim time
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # ----- NEW: Service bridge (create/remove/set_pose/control) -----
    service_bridge_node = TimerAction(
        period=4.0,  # give Ignition a few seconds to bring up services
        actions=[
            ExecuteProcess(
                cmd=['echo', '🔗 DEBUG: starting ros_gz service bridges (create/remove/set_pose/control)'],
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
        ]
    )

    # ----- NEW: Pose/info + odom bridge -----
    pose_and_odom_bridge = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['echo', '🔗 DEBUG: starting odom + world pose/info bridges (delayed)'],
                output='screen'
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='pose_and_odom_bridge',
                arguments=[
                    # bridge odom
                    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    # bridge world pose info (Pose_V -> TFMessage)
                    '/world/forest_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                ],
                output='screen'
            )
        ]
    )
    
    # Custom TurtleBot4 spawn - bypass problematic built-in spawner
    turtlebot4_spawn_launch = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['echo', '🤖 DEBUG: Starting CUSTOM robot spawn at 8 seconds (bypassing built-in spawner)...'],
                output='screen'
            ),
            # Robot description - use direct robot_state_publisher with processed URDF
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
            # Direct robot spawning without ROS2 control spawner
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
            # Essential bridges only (skip the problematic controller spawner)
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='essential_cmd_vel_bridge',
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
                ],
                output='screen'
            ),
            # Model-specific cmd_vel bridge for TurtleBot4
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='model_cmd_vel_bridge',
                arguments=[
                    '/model/turtlebot4/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
                ],
                output='screen'
            ),
        ]
    )
    
    # Robot already has direct DiffDrive control - no additional spawners needed
    # The TurtleBot4 uses built-in Ignition DiffDrive plugin, not ROS2 control
    
    # Additional bridges for enhanced functionality
    lidar_bridge_node = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['echo', '� DEBUG: Starting LiDAR bridge at 10 seconds...'],
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
        ]
    )
    
    # Camera bridge node - automatically bridge camera topic for easy access
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
                    '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                    '--ros-args', '-r', '/camera:=/test_camera'
                ],
                output='screen'
            )
        ]
    )


    # Add explicit cmd_vel bridges for both topics
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )
    model_cmd_vel_bridge_explicit = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='model_cmd_vel_bridge_explicit',
        arguments=['/model/turtlebot4/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    return LaunchDescription([
        # Environment variables
        ign_gazebo_resource_path_env,
        ign_gazebo_system_plugin_path_env,
        ldn_library_path_env,
        gz_plugin_path_env,
        rmw_implementation_env,
        
        # Launch arguments
        declare_use_sim_time_cmd,
        declare_model_cmd,
        declare_x_position_cmd,
        declare_y_position_cmd,
        declare_z_position_cmd,
        declare_yaw_cmd,
        
        # Launch sequence: World -> Clock -> Service & Pose/Odom bridges -> Custom Robot Spawn -> LiDAR -> Camera
        turtlebot4_world_launch,
        clock_bridge,
        service_bridge_node,
        pose_and_odom_bridge,
        turtlebot4_spawn_launch,
        lidar_bridge_node,
        camera_bridge_node,
        cmd_vel_bridge,
        model_cmd_vel_bridge_explicit,
    ])
