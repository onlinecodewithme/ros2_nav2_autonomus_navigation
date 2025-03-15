#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='tracked_robot_world.sdf',
        description='World file to load in Gazebo'
    )
    
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='keyboard',
        description='Control mode for the robot (keyboard, joystick, autonomous)'
    )
    
    use_zed_arg = DeclareLaunchArgument(
        'use_zed',
        default_value='false',
        description='Enable ZED camera if true'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Enable RViz visualization if true (may crash on some systems)'
    )
    
    use_nav_arg = DeclareLaunchArgument(
        'use_nav',
        default_value='true',
        description='Enable navigation stack if true'
    )
    
    # LaunchConfiguration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    control_mode = LaunchConfiguration('control_mode')
    use_zed = LaunchConfiguration('use_zed')
    use_rviz = LaunchConfiguration('use_rviz')
    use_nav = LaunchConfiguration('use_nav')
    
    # Get package directories
    pkg_share = get_package_share_directory('tracked_robot_sim')
    nav_pkg_share = get_package_share_directory('tracked_robot_nav')
    
    # Paths
    urdf_path = os.path.join(pkg_share, 'urdf', 'tracked_robot.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'tracked_robot_world.sdf')
    rviz_config_path = os.path.join(nav_pkg_share, 'config', 'slam.rviz')  # Use the navigation RViz config
    zed_config_path = os.path.join(pkg_share, 'config', 'zed2i_config.yaml')
    model_path = os.path.join(pkg_share, 'models', 'tracked_robot', 'model.sdf')
    nav2_params_path = os.path.join(nav_pkg_share, 'config', 'nav2_params.yaml')
    
    # Load the URDF file
    try:
        with open(urdf_path, 'r') as urdf_file:
            robot_description = urdf_file.read()
    except:
        robot_description = '<?xml version="1.0"?><robot name="tracked_robot"><link name="base_link"/></robot>'
    
    # Gazebo launch with Gazebo Garden (Ignition Gazebo)
    gz_sim_launch = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path, '--force-version', '6'],
        output='screen'
    )
    
    # Bridge to connect ROS and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/tracked_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/tracked_robot/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/zed2i/rgb@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Spawn robot in Gazebo Garden
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tracked_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-file', model_path
        ],
        output='screen'
    )
    
    # ZED 2i camera node - only launched if use_zed is true
    zed_node = Node(
        package='zed_wrapper',
        executable='zed_wrapper_node',
        name='zed2i_node',
        output='screen',
        parameters=[zed_config_path],
        condition=IfCondition(use_zed)
    )
    
    # RViz - only launched if use_rviz is true
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    # Tracked robot controller
    track_controller = Node(
        package='tracked_robot_sim',
        executable='track_controller_node.py',
        name='track_controller',
        output='screen',
        parameters=[
            {'control_mode': control_mode},
            {'use_sim_time': use_sim_time},
            {'max_linear_speed': 1.0},
            {'max_angular_speed': 2.0}
        ]
    )
    
    # Load joystick driver if in joystick mode
    joystick_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", control_mode, "'=='joystick'"]))
    )
    
    # Navigation nodes - only launched if use_nav is true
    
    # SLAM Toolbox for mapping
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'map_frame': 'map'},
            {'base_frame': 'base_link'},
            {'odom_frame': 'odom'},
            {'scan_topic': '/scan'}
        ],
        condition=IfCondition(use_nav)
    )
    
    # Local costmap for navigation
    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[nav2_params_path],
        condition=IfCondition(use_nav)
    )
    
    # Global costmap for navigation
    global_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[nav2_params_path],
        condition=IfCondition(use_nav)
    )
    
    # Laser scan from point cloud for navigation
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'target_frame': 'base_link'},
            {'transform_tolerance': 0.01},
            {'min_height': 0.0},
            {'max_height': 1.0},
            {'angle_min': -1.5708},  # -M_PI/2
            {'angle_max': 1.5708},   # M_PI/2
            {'angle_increment': 0.0087},  # M_PI/360.0
            {'scan_time': 0.3333},
            {'range_min': 0.45},
            {'range_max': 4.0},
            {'use_inf': True},
            {'inf_epsilon': 1.0},
            # Fix QoS issues - use best_effort to match RViz
            {'qos_reliability': 'best_effort'},
            {'qos_durability': 'volatile'}
        ],
        remappings=[
            ('cloud_in', '/zed2i/rgb/points'),
            ('scan', '/scan')
        ],
        condition=IfCondition(use_nav)
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        world_file_arg,
        control_mode_arg,
        use_zed_arg,
        use_rviz_arg,
        use_nav_arg,
        gz_sim_launch,
        bridge,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        zed_node,
        rviz_node,
        track_controller,
        joystick_node,
        slam_toolbox_node,
        local_costmap_node,
        global_costmap_node,
        pointcloud_to_laserscan_node
    ])
