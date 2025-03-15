import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_share = get_package_share_directory('tracked_robot_simulation')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # Create the robot_localization node
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'frequency': 30.0,
                'sensor_timeout': 0.1,
                'two_d_mode': True,
                'publish_tf': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                'odom0': 'odom',
                'odom0_config': [
                    True, True, False,  # x, y, z
                    False, False, True,  # roll, pitch, yaw
                    True, True, False,  # vx, vy, vz
                    False, False, True,  # vroll, vpitch, vyaw
                    False, False, False  # ax, ay, az
                ],
                'odom0_queue_size': 10,
                'odom0_differential': False,
                'odom0_relative': False,
                'imu0': 'imu',
                'imu0_config': [
                    False, False, False,  # x, y, z
                    True, True, True,  # roll, pitch, yaw
                    False, False, False,  # vx, vy, vz
                    True, True, True,  # vroll, vpitch, vyaw
                    True, True, True  # ax, ay, az
                ],
                'imu0_queue_size': 10,
                'imu0_differential': False,
                'imu0_relative': False,
                'print_diagnostics': True,
                'debug': False,
                'debug_out_file': '/tmp/ekf_debug.txt'
            }
        ]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add the nodes to the launch description
    ld.add_action(robot_localization_node)
    
    return ld
