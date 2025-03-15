import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
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
    
    # Log info for debugging
    log_gazebo = LogInfo(msg="Starting Gazebo...")
    
    # Gazebo Garden launch via ros-gz
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', default_world_path],
        output='screen'
    )
    
    # Log info for debugging
    log_robot_state = LogInfo(msg="Starting Robot State Publisher...")
    
    # Robot state publisher
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(description_pkg_dir, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )
    
    # Log info for debugging
    log_spawn = LogInfo(msg="Spawning robot in Gazebo...")
    
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
    
    # Log info for debugging
    log_bridge = LogInfo(msg="Starting ROS-Gazebo bridge...")
    
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
    
    # Log info for debugging
    log_laser = LogInfo(msg="Starting simulated laser scan...")
    
    # Simulated laser scan for visualization
    simulated_laser = Node(
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
        output='screen'
    )
    
    # Fake laser scan publisher for testing
    fake_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fake_laser_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'laser_frame'],
        output='screen'
    )
    
    # Log info for debugging
    log_static_tf = LogInfo(msg="Publishing static transforms...")
    
    # Publish static transforms for debugging - connect the TF tree
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # Add a direct static transform from map to base_link for debugging
    static_tf_map_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )
    
    # Add transform from base_link to camera_link
    static_tf_base_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_camera',
        arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'
    )
    
    # Add transforms for left and right tracks
    static_tf_base_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_left',
        arguments=['0', '0.175', '0', '0', '0', '0', 'base_link', 'left_track'],
        output='screen'
    )
    
    static_tf_base_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_right',
        arguments=['0', '-0.175', '0', '0', '0', '0', 'base_link', 'right_track'],
        output='screen'
    )
    
    # Log info for debugging
    log_map = LogInfo(msg="Starting map publisher...")
    
    # Create a simple map for visualization
    map_publisher = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': '',
            'topic_name': 'map',
            'frame_id': 'map',
            'resolution': 0.05
        }]
    )
    
    # Create a simple costmap for visualization
    fake_costmap_publisher = Node(
        package='nav2_map_server',
        executable='map_server',
        name='costmap_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': '',
            'topic_name': 'global_costmap/costmap',
            'frame_id': 'map',
            'resolution': 0.05
        }]
    )
    
    # Log info for debugging
    log_rviz = LogInfo(msg="Starting RViz...")
    
    # RViz with simplified configuration for debugging
    rviz_config_file = os.path.join(sim_pkg_dir, 'config', 'debug.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    # Log info for debugging
    log_tf_echo = LogInfo(msg="Starting TF echo for debugging...")
    
    # TF echo for debugging
    tf_echo = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'tf2_echo', 'map', 'base_link'],
        output='screen'
    )
    
    # Log info for debugging
    log_topic_list = LogInfo(msg="Listing topics for debugging...")
    
    # Topic list for debugging
    topic_list = ExecuteProcess(
        cmd=['ros2', 'topic', 'list'],
        output='screen'
    )
    
    # Log info for debugging
    log_odom_tf = LogInfo(msg="Starting Odometry to TF node...")
    
    # Odometry to TF node
    odom_to_tf_node = Node(
        package='tracked_robot_simulation',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': '/odom',
            'parent_frame': 'map',
            'child_frame': 'base_link'
        }]
    )
    
    # Log info for debugging
    log_obstacles = LogInfo(msg="Starting Gazebo obstacle visualizer...")
    
    # Gazebo obstacle visualizer node
    gazebo_obstacle_visualizer = Node(
        package='tracked_robot_simulation',
        executable='gazebo_obstacle_visualizer',
        name='gazebo_obstacle_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'marker_topic': '/gazebo_obstacles',
            'update_rate': 10.0,
            'marker_lifetime': 0.5,
            'marker_color_r': 0.8,
            'marker_color_g': 0.2,
            'marker_color_b': 0.2,
            'marker_color_a': 0.8
        }]
    )
    
    # Log info for debugging
    log_teleop = LogInfo(msg="Starting teleop node for robot control...")
    
    # Teleop node for controlling the robot with keyboard
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # Run in a separate terminal window
        remappings=[('/cmd_vel', '/cmd_vel')]
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        # Debugging logs
        log_gazebo,
        gazebo,
        
        log_robot_state,
        robot_state_publisher_launch,
        
        log_spawn,
        spawn_entity,
        
        log_bridge,
        bridge,
        
        log_laser,
        simulated_laser,
        fake_laser,
        
        log_static_tf,
        static_tf_map_odom,
        static_tf_map_base,  # Direct static transform from map to base_link
        static_tf_base_camera,
        static_tf_base_left,
        static_tf_base_right,
        
        # Commenting out the odom_to_tf node for now since we're using a static transform
        # log_odom_tf,
        # odom_to_tf_node,
        
        log_map,
        map_publisher,
        fake_costmap_publisher,
        
        log_rviz,
        rviz_node,
        
        log_tf_echo,
        tf_echo,
        
        log_topic_list,
        topic_list,
        
        log_obstacles,
        gazebo_obstacle_visualizer,
        
        log_teleop,
        teleop_node,
    ])
