import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Add launch argument for remote RViz
    use_remote_rviz_arg = DeclareLaunchArgument(
        'use_remote_rviz',
        default_value='false',
        description='Use RViz on a remote machine instead of locally'
    )
    
    use_remote_rviz = LaunchConfiguration('use_remote_rviz')
    
    # Get the launch directory
    nav_dir = get_package_share_directory('tracked_robot_nav')
    
    # Set environment variable for display
    set_display = SetEnvironmentVariable(
        name='DISPLAY',
        value=':1'
    )
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Use the map from tracked_robot_nav/maps
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(nav_dir, 'maps', 'map.yaml'))
    
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
        'base_frame': 'base_footprint'
    }
    
    # Exploration parameters
    explore_params = {
        'robot_base_frame': 'base_footprint',
        'costmap_topic': '/map',
        'costmap_updates_topic': '/map_updates',
        'visualize': True,
        'planner_frequency': 0.15,
        'progress_timeout': 30.0,
        'potential_scale': 3.0,
        'orientation_scale': 0.0,
        'gain_scale': 1.0,
        'transform_tolerance': 0.3,
        'min_frontier_size': 0.75
    }

    # Define nodes list
    nodes = [
        # Add the launch argument
        use_remote_rviz_arg,
        
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
        
        # Launch SLAM for mapping
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'map_frame': 'map'},
                {'base_frame': 'base_footprint'},
                {'odom_frame': 'odom'},
                {'scan_topic': '/zed2i/zed_node/point_cloud/cloud_registered'}
            ],
        ),
        
        # Load the nav2_params.yaml file for costmap configuration
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[os.path.join(nav_dir, 'config', 'nav2_params.yaml')],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),
        
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            output='screen',
            parameters=[os.path.join(nav_dir, 'config', 'nav2_params.yaml')],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),
        
        # Launch lifecycle manager for the costmap nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmaps',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': []}
            ]
        ),
        
        # Launch explore_lite for autonomous exploration
        Node(
            package='explore_lite',
            executable='explore',
            name='explore',
            output='screen',
            parameters=[explore_params]
        ),
        
        # Launch RViz for visualization only if not using remote RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(nav_dir, 'config', 'slam.rviz')] 
                if os.path.exists(os.path.join(nav_dir, 'config', 'slam.rviz')) 
                else [],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=UnlessCondition(use_remote_rviz)
        ),
    ]
    
    return LaunchDescription(nodes)
