import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
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
    
    # Set environment variable for display
    set_display = SetEnvironmentVariable(
        name='DISPLAY',
        value=':1'
    )
    
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
    
    # Launch controllers
    cmd_vel_to_tracks = Node(
        package='tracked_robot_controller',
        executable='cmd_vel_to_tracks',
        name='cmd_vel_to_tracks',
        output='screen',
        parameters=[controller_params]
    )
    
    track_odometry = Node(
        package='tracked_robot_controller',
        executable='track_odometry',
        name='track_odometry',
        output='screen',
        parameters=[odom_params]
    )
    
    # Launch robot description (URDF)
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('tracked_robot_description'),
            '/launch/robot_state_publisher.launch.py'
        ]),
    )
    
    # Launch ZED camera
    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('zed_navigation_config'),
            '/launch/zed2i_nav.launch.py'
        ])
    )
    
    # Get the launch directory
    nav_dir = get_package_share_directory('tracked_robot_nav')
    
    # Use the map from our project
    map_file = os.path.join(nav_dir, 'maps', 'map.yaml')
    
    # Pass the map file to the navigation launch
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('tracked_robot_nav'),
            '/launch/navigation.launch.py'
        ]),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false'
        }.items()
    )
    
    # Define nodes list
    nodes = [
        # Add the launch argument
        use_remote_rviz_arg,
        
        set_display,
        cmd_vel_to_tracks,
        track_odometry,
        robot_description,
        zed_camera,
        navigation,
        
        # RViz is launched separately using the run_rviz_map.sh script
    ]
    
    return LaunchDescription(nodes)
