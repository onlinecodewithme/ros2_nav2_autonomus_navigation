import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    nav_dir = get_package_share_directory('tracked_robot_nav')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    # Use the existing map file with absolute path
    map_file = os.path.join('/home/x4/ros2_ws/install/tracked_robot_nav/share/tracked_robot_nav/maps', 'map.yaml')
    
    # Use the simplified costmap configuration
    costmap_params_file = os.path.join(nav_dir, 'config', 'costmap_test.yaml')
    
    # Set environment variable for display
    set_display = SetEnvironmentVariable(
        name='DISPLAY',
        value=':1'
    )
    
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        set_display,
        
        # Launch robot description (URDF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('tracked_robot_description'),
                '/launch/robot_state_publisher.launch.py'
            ]),
        ),
        
        # Launch costmap filter info
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                nav_dir,
                '/launch/costmap_filter_info.launch.py'
            ]),
        ),
        
        # Launch ZED camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('zed_navigation_config'),
                '/launch/zed2i_nav.launch.py'
            ]),
        ),
        
        # Launch Nav2 stack for costmaps only
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_file}
            ]
        ),
        
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap_node',
            output='screen',
            parameters=[costmap_params_file]
        ),
        
        # Launch lifecycle manager for the map server and costmap node
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),
        
        # Launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(nav_dir, 'config', 'costmap_test.rviz')],
            output='screen'
        ),
    ])
