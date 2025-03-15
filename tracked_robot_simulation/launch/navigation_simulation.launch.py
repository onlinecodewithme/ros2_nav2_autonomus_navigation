import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare(package='tracked_robot_simulation').find('tracked_robot_simulation')
    nav_pkg_share = FindPackageShare(package='tracked_robot_nav').find('tracked_robot_nav')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_slam = LaunchConfiguration('slam', default='true')
    map_yaml_file = LaunchConfiguration('map', default='')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Whether to run SLAM (true) or localization with existing map (false)')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load')
    
    # Include the robot spawning launch file
    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Include either SLAM or Navigation based on the 'slam' argument
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav_pkg_share,
            '/launch/slam.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file
        }.items(),
        condition=IfCondition(use_slam)
    )
    
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav_pkg_share,
            '/launch/navigation.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file
        }.items(),
        condition=UnlessCondition(use_slam)
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    
    # Add the commands to the launch description
    ld.add_action(spawn_robot_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(navigation_cmd)
    
    return ld
