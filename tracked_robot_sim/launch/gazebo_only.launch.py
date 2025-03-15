#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
    pkg_share = get_package_share_directory('tracked_robot_sim')
    
    # Paths
    world_path = os.path.join(pkg_share, 'worlds', 'tracked_robot_world.sdf')
    model_path = os.path.join(pkg_share, 'models', 'tracked_robot', 'model.sdf')
    
    # Gazebo launch with Gazebo Garden (Ignition Gazebo)
    gz_sim_launch = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path, '--force-version', '6', '--verbose'],
        output='screen'
    )
    
    # Spawn robot in Gazebo Garden
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tracked_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-file', model_path,
            '-verbose'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        gz_sim_launch,
        spawn_robot
    ])
