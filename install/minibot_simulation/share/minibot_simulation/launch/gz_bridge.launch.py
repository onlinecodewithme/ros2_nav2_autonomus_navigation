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
    world_file = os.path.join(pkg_minibot_simulation, 'worlds', 'empty.sdf')
    
    # Gazebo simulation server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )
    
    # Bridge
    # Bridge with debug output
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock bridge
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Image bridge - use the exact topic name from Gazebo
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # TF bridge
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '--ros-args', '--log-level', 'debug'
        ],
        remappings=[
            ('/camera/image', '/camera/image_raw'),
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_minibot_simulation, 'rviz', 'sim.rviz')],
        output='screen'
    )
    
    # Image viewer (additional way to view camera feed)
    image_view = Node(
        package='image_view',
        executable='image_view',
        name='image_view',
        remappings=[
            ('image', '/camera/image_raw'),
        ],
        output='screen'
    )

    image_transport = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        arguments=['raw', 'in:=/camera/image_raw', 'out:=/camera/image_republished'],
        output='screen'
    )
    
    # Note: We're removing the TF broadcaster node since it requires
    # a script in the lib directory that doesn't exist yet
    
    return LaunchDescription([
        gzserver,
        bridge,
        rviz,
        image_view,
        image_transport
    ])