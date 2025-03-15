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
    map_yaml_file = os.path.join(pkg_share, 'maps', 'map.yaml')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # Include the robot spawning launch file
    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Include the navigation launch file
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav_pkg_share,
            '/launch/navigation.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file
        }.items()
    )
    
    # Include the RViz launch file
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Create the autonomous navigation node
    autonomous_navigation_node = Node(
        package='tracked_robot_simulation',
        executable='autonomous_navigation.py',
        name='autonomous_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    
    # Include the robot_localization launch file
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Add the commands to the launch description
    ld.add_action(spawn_robot_cmd)
    ld.add_action(robot_localization_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(autonomous_navigation_node)
    
    return ld
