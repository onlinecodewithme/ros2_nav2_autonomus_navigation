import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    nav_dir = get_package_share_directory('tracked_robot_nav')
    
    # Set environment variable for display
    set_display = SetEnvironmentVariable(
        name='DISPLAY',
        value=':1'
    )
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = os.path.join(nav_dir, 'maps', 'map.yaml')
    
    # Check if map file exists
    if not os.path.exists(map_file):
        print(f"Warning: Map file {map_file} does not exist!")
    
    # Use our simple map config
    rviz_config_file = os.path.join(nav_dir, 'config', 'simple_map.rviz')
    if not os.path.exists(rviz_config_file):
        print(f"Warning: RViz config file {rviz_config_file} does not exist!")
        rviz_config_file = ""
    
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        set_display,
        
        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename': map_file}]
        ),
        
        # Lifecycle manager for map server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file] if rviz_config_file else [],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
