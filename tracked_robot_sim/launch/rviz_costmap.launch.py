#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    # LaunchConfiguration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get package directories
    nav_pkg_share = get_package_share_directory('tracked_robot_nav')
    
    # Paths
    rviz_config_path = os.path.join(nav_pkg_share, 'config', 'slam.rviz')
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        rviz_node
    ])
