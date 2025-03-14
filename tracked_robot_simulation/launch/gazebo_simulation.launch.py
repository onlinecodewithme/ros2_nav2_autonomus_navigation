import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
    
    # RViz
    rviz_config_file = os.path.join(sim_pkg_dir, 'config', 'simulation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
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
            
        # Launch components
        gazebo,
        robot_state_publisher_launch,
        spawn_entity,
        bridge,
        rviz_node,
    ])
