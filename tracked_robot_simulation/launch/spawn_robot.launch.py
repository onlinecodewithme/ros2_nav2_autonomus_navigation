import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare(package='tracked_robot_simulation').find('tracked_robot_simulation')
    
    # Set the path to the URDF file
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'tracked_robot_gazebo.urdf.xacro')
    
    # Set the path to the world file
    world_file_path = os.path.join(pkg_share, 'worlds', 'obstacle_world.world')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x-pose of the robot')
    
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial y-pose of the robot')
    
    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Initial z-pose of the robot')
    
    # Create the robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file_path]),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Create the joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create the joint state publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Launch Ignition Gazebo with software rendering
    ignition_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '--render-engine', 'ogre', world_file_path],
        output='screen'
    )
    
    # Spawn the robot in Ignition Gazebo
    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tracked_robot',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )
    
    # Create the ROS-Ignition bridge
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock bridge
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            
            # Cmd vel bridge - make sure direction is correct
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            
            # Odometry bridge
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            
            # LaserScan bridge
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            
            # Camera bridge
            '/zed2i/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed2i/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            
            # IMU bridge
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            
            # Joint states bridge
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            
            # TF bridge
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            
            # Marker bridge for visualization
            '/visualization_marker@visualization_msgs/msg/Marker[ignition.msgs.Marker',
            '/visualization_marker_array@visualization_msgs/msg/MarkerArray[ignition.msgs.Marker_V'
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen'
    )
    
    # Static transform publishers removed to avoid conflicts with robot_localization and Gazebo
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)
    
    # Add the nodes to the launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(ignition_gazebo)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(bridge_cmd)
    # Static transform publishers removed
    
    return ld
