import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
    keepout_params_file = os.path.join(nav_dir, 'config', 'keepout_nav2_params.yaml')
    map_file = os.path.join(nav_dir, 'maps', 'map_keepout.yaml')
    
    lifecycle_nodes = ['map_server', 
                       'amcl',
                       'planner_server',
                       'controller_server',
                       'behavior_server',
                       'bt_navigator',
                       'filter_mask_server',
                       'costmap_filter_info_server'
                      ]
    
    # Controller parameters
    controller_params = {
        'wheel_separation': 0.3,
        'max_linear_speed': 0.5,
        'max_angular_speed': 1.0,
        'max_pwm': 255
    }
    
    # Odometry parameters
    odom_params = {
        'wheel_separation': 0.3,
        'encoder_resolution': 360.0,
        'wheel_radius': 0.05,
        'odom_frame': 'odom',
        'base_frame': 'base_link'
    }

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        set_display,
        
        # Launch controllers
        Node(
            package='tracked_robot_controller',
            executable='cmd_vel_to_tracks',
            name='cmd_vel_to_tracks',
            output='screen',
            parameters=[controller_params]
        ),
        
        Node(
            package='tracked_robot_controller',
            executable='track_odometry',
            name='track_odometry',
            output='screen',
            parameters=[odom_params]
        ),
        
        # Launch robot description (URDF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('tracked_robot_description'),
                '/launch/robot_state_publisher.launch.py'
            ]),
        ),
        
        # Launch ZED camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('zed_navigation_config'),
                '/launch/zed2i_nav.launch.py'
            ]),
        ),
        
        # Costmap filter info server for keepout zones
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'filter_info_topic': 'costmap_filter_info'},
                        {'type': 0},
                        {'filter_info_frame': 'map'},
                        {'base_frame': 'base_link'},
                        {'transform_tolerance': 0.1}]
        ),
        
        # Filter mask server for keepout zones
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': map_file},
                        {'topic_name': 'keepout_filter_mask'},
                        {'frame_id': 'map'}]
        ),
        
        # Map server - explicitly set the map file path
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename': os.path.join(nav_dir, 'maps', 'map_keepout.yaml')}]
        ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[keepout_params_file]
        ),
                     
        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[keepout_params_file]
        ),

        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[keepout_params_file]
        ),
            
        # Behaviors server (formerly recoveries)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[keepout_params_file],
            output='screen'
        ),

        # BT navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[keepout_params_file]
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
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(nav_dir, 'config', 'navigation.rviz')] 
                if os.path.exists(os.path.join(nav_dir, 'config', 'navigation.rviz')) 
                else [],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
