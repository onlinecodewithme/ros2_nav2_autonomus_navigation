import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='true')
    map_file = LaunchConfiguration('map', default='')
    
    # Get package directories
    sim_pkg_dir = get_package_share_directory('tracked_robot_simulation')
    nav_pkg_dir = get_package_share_directory('tracked_robot_nav')
    
    # Gazebo simulation launch
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(sim_pkg_dir, 'launch', 'gazebo_simulation.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )
    
    # Nav2 parameters
    nav2_params_path = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')
    
    # SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav_pkg_dir, 'launch', 'slam.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )
    
    # Simulated laser scan from depth camera
    # This simulates a laser scan from the depth camera for navigation
    simulated_laser_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[{
            'scan_time': 0.033,
            'range_min': 0.1,
            'range_max': 10.0,
            'scan_height': 1,
            'output_frame': 'camera_link',
        }],
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan')
        ],
    )
    
    # Autonomous exploration node
    # This node will automatically explore the unknown environment
    exploration_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_lite',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_base_frame': 'base_link',
            'costmap_topic': '/global_costmap/costmap',
            'costmap_updates_topic': '/global_costmap/costmap_updates',
            'visualize': True,
            'planner_frequency': 0.15,
            'progress_timeout': 30.0,
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.75
        }],
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'world_file',
            default_value='unknown_environment.world',
            description='Name of the world file to load'),
            
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Whether to run SLAM (true) or localization with existing map (false)'),
            
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Path to map file to load'),
            
        # Launch components
        gazebo_sim,
        slam_launch,
        simulated_laser_scan,
        exploration_node,
    ])
