import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(nav_dir, 'maps', 'map.yaml'))
    
    nav2_params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(nav_dir, 'config', 'nav2_params.yaml'))

    lifecycle_nodes = ['map_server', 
                       'amcl',
                       'planner_server',
                       'controller_server',
                       'behavior_server',
                       'bt_navigator'
                      ]
    
    # RViz is launched separately using the run_rviz_map.sh script
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
                        {'yaml_filename': map_dir}]
        ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_file]
        ),
                     
        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file]
        ),

        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file]
        ),
            
        # Behaviors server (formerly recoveries)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[nav2_params_file],
            output='screen'
        ),

        # BT navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file]
        ),
            
        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]
        )
    ])
