import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map', default='')
    use_slam = LaunchConfiguration('slam', default='true')
    zed_dir = get_package_share_directory('zed_wrapper')
    nav_dir = get_package_share_directory('tracked_robot_nav')
    
    # Controller parameters
    controller_params = {
        'wheel_separation': 0.3,
        'max_linear_speed': 0.5,
        'max_angular_speed': 1.0,
        'max_pwm': 255
    }
    
    # Odometry parameters
    odom_params = {
        'wheel_separation': 0.3,
        'encoder_resolution': 360.0,
        'wheel_radius': 0.05,
        'odom_frame': 'odom',
        'base_frame': 'base_link'
    }
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Whether to run SLAM (true) or localization with existing map (false)'),
        
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Path to map file to load'),
            
        # Launch controllers
        Node(
            package='tracked_robot_controller',
            executable='cmd_vel_to_tracks',
            name='cmd_vel_to_tracks',
            output='screen',
            parameters=[controller_params]
        ),
        
        Node(
            package='tracked_robot_controller',
            executable='track_odometry',
            name='track_odometry',
            output='screen',
            parameters=[odom_params]
        ),
        
        # Launch robot description (URDF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('tracked_robot_description'),
                '/launch/robot_state_publisher.launch.py'
            ]),
        ),
        
    # Launch ZED camera configuration
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('zed_navigation_config'),
            '/launch/zed2i_nav.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    ),
        
        # Launch either SLAM or Navigation based on argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                nav_dir,
                '/launch/slam.launch.py' if use_slam else '/launch/navigation.launch.py'
            ]),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': use_sim_time
            }.items(),
        ),
    ])
