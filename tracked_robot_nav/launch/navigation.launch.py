import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('tracked_robot_nav'),
            'maps',
            'map.yaml'))
    
    nav2_params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('tracked_robot_nav'),
            'config',
            'nav2_params.yaml'))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    robot_description_dir = get_package_share_directory('tracked_robot_description')
    
    return LaunchDescription([
        # Launch robot state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_dir, '/launch/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        
        # Launch ZED camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('zed_navigation_config'),
                '/launch/zed2i_nav.launch.py'
            ]),
        ),
        
        # Launch Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file
            }.items(),
        ),
    ])