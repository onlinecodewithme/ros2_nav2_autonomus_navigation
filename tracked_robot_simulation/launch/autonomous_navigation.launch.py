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
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # Define slam parameter
    slam = LaunchConfiguration('slam', default='false')
    
    # Declare slam argument
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Whether to run SLAM (true) or localization with existing map (false)')
    
    # Include the simulation launch file
    simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulation_complete.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': slam,  # Use localization with existing map
            'map': os.path.join(pkg_share, 'maps', 'map.yaml')  # Use a pre-built map
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
    ld.add_action(declare_slam_cmd)
    
    # Add the commands to the launch description
    ld.add_action(simulation_cmd)
    ld.add_action(autonomous_navigation_node)
    
    return ld
