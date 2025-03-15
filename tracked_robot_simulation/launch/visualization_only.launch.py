import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Get package directories
    sim_pkg_dir = get_package_share_directory('tracked_robot_simulation')
    description_pkg_dir = get_package_share_directory('tracked_robot_description')
    
    # Default world path
    default_world_path = os.path.join(sim_pkg_dir, 'worlds', 'unknown_environment.world')
    
    # Gazebo Garden launch via ros-gz
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', default_world_path],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(description_pkg_dir, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )
    
    # Spawn the robot in Gazebo Garden
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-entity', 'tracked_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen',
    )
    
    # Bridge to connect ROS 2 and Gazebo Garden
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/camera/depth/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen'
    )
    
    # Add odom to tf node
    odom_to_tf_node = Node(
        package='tracked_robot_simulation',
        executable='odom_to_tf',
        name='odom_to_tf',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': '/odom',
            'parent_frame': 'odom',
            'child_frame': 'base_link'
        }],
        output='screen'
    )
    
    # Simulated laser scan from depth camera
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
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan')
        ],
    )
    
    # SLAM Toolbox node with simulation parameters
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': 'map',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'scan_topic': '/scan',
            'mode': 'mapping',  # Explicitly set to mapping mode
            'debug_logging': True,
            'publish_period': 0.1  # More frequent updates
        }],
    )
    
    # RViz with custom configuration for simulation
    rviz_config_file = os.path.join(sim_pkg_dir, 'config', 'full_simulation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    # Simple map publisher
    map_publisher = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': ''
        }]
    )
    
    # Lifecycle manager for map server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        # Launch components
        gazebo,
        robot_state_publisher_launch,
        spawn_entity,
        bridge,
        odom_to_tf_node,
        simulated_laser_scan,
        slam_toolbox_node,
        rviz_node,
    ])
