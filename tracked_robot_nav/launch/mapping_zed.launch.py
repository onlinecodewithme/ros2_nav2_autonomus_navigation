import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Add launch argument for remote RViz
    use_remote_rviz_arg = DeclareLaunchArgument(
        'use_remote_rviz',
        default_value='false',
        description='Use RViz on a remote machine instead of locally'
    )
    
    use_remote_rviz = LaunchConfiguration('use_remote_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    nav_dir = get_package_share_directory('tracked_robot_nav')
    zed_dir = get_package_share_directory('zed_wrapper')
    
    # We need to provide a map file path for Nav2
    # Check if the example map exists and use it if available
    example_map_file = '/home/x4/ros2_ws/src/example_project/agv_proto/maps/my_home_map.yaml'
    if os.path.exists(example_map_file):
        map_file = example_map_file
    else:
        map_file = os.path.join(nav_dir, 'maps', 'map.yaml')
    
    # Load the nav2_params.yaml file for costmap configuration
    nav2_params_file = os.path.join(nav_dir, 'config', 'nav2_params.yaml')
    
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
    
    # Define nodes list
    nodes = [
        # Add the launch argument
        use_remote_rviz_arg,
        
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
        
        # Launch ZED camera using the official launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('zed_navigation_config'),
                '/launch/zed2i_nav.launch.py'
            ]),
        ),
        
        # Launch SLAM Toolbox for mapping
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
        
        # Launch only the map server from Nav2 stack
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
        
        # Add global costmap node
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[nav2_params_file],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),
        
        # Add local costmap node
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            output='screen',
            parameters=[nav2_params_file],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),
        
        # Add controller server for path planning
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file],
            remappings=[('/cmd_vel', '/cmd_vel')]
        ),
        
        # Add planner server for path planning
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file]
        ),
        
        # Add recoveries server
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_params_file]
        ),
        
        # Add bt_navigator for behavior tree navigation
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file]
        ),
        
        # Launch lifecycle manager for all navigation nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': [
                    'map_server',
                    'global_costmap',
                    'local_costmap',
                    'controller_server',
                    'planner_server',
                    'recoveries_server',
                    'bt_navigator'
                ]}
            ]
        ),
        
        # Launch RViz for visualization only if not using remote RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(nav_dir, 'config', 'slam.rviz')],
            output='screen',
            condition=UnlessCondition(use_remote_rviz)
        ),
    ]
    
    return LaunchDescription(nodes)
