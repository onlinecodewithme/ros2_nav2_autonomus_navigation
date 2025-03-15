#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_minibot_simulation = get_package_share_directory('minibot_simulation')
    
    # Path to the world file
    world_file = os.path.join(pkg_minibot_simulation, 'worlds', 'simplified_crowded_world.sdf')
    
    # Gazebo simulation server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )
    
    # Bridge for multiple camera topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock bridge
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Camera bridges
            '/camera_top/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_side/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/camera_top/image', '/camera_top/image_raw'),
            ('/camera_front/image', '/camera_front/image_raw'),
            ('/camera_side/image', '/camera_side/image_raw'),
        ],
        output='screen',
    )
    
    # Image view for top camera
    image_view_top = Node(
        package='image_view',
        executable='image_view',
        name='image_view_top',
        remappings=[
            ('image', '/camera_top/image_raw'),
        ],
        output='screen'
    )
    
    # Image view for front camera
    image_view_front = Node(
        package='image_view',
        executable='image_view',
        name='image_view_front',
        remappings=[
            ('image', '/camera_front/image_raw'),
        ],
        output='screen'
    )
    
    # Image view for side camera
    image_view_side = Node(
        package='image_view',
        executable='image_view',
        name='image_view_side',
        remappings=[
            ('image', '/camera_side/image_raw'),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gzserver,
        bridge,
        image_view_top,
        image_view_front,
        image_view_side,
    ])