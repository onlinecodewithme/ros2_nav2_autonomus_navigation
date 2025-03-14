import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    nav_dir = get_package_share_directory('tracked_robot_nav')
    zed_dir = get_package_share_directory('zed_wrapper')
    
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
                zed_dir,
                '/launch/zed_camera.launch.py'  # Adjust this filename if needed
            ]),
            launch_arguments={
                'camera_model': 'zed2i',
                'camera_name': 'zed2i',
                'publish_tf': 'true',
                'publish_map_tf': 'true',
                'mapping': 'true'
            }.items()
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
                {'base_frame': 'base_link'},
                {'odom_frame': 'odom'},
                {'scan_topic': '/zed2i/zed_node/point_cloud/cloud_registered'}
            ],
        )
        ,
        
        # Launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(nav_dir, 'config', 'slam.rviz')],
            output='screen'
        ),
    ])