import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='true')
    
    # Get package directories
    sim_pkg_dir = get_package_share_directory('tracked_robot_simulation')
    description_pkg_dir = get_package_share_directory('tracked_robot_description')
    nav_pkg_dir = get_package_share_directory('tracked_robot_nav')
    
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
    
    # SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav_pkg_dir, 'launch', 'slam.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )
    
    # Nav2 parameters
    nav2_params_path = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')
    
    # Nav2 launch
    nav2_bringup = Node(
        package='nav2_bringup',
        executable='bringup_launch.py',
        name='nav2_bringup',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            nav2_params_path
        ],
    )
    
    # Create a modified SLAM params file for simulation
    slam_params_sim = {
        'slam_toolbox': {
            'ros__parameters': {
                'use_sim_time': True,
                'map_frame': 'map',
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'scan_topic': '/scan',  # Use simulated laser scan
                'mode': 'mapping',
                'map_update_interval': 5.0,
                'resolution': 0.05,
                'max_laser_range': 20.0,
                'minimum_travel_distance': 0.5,
                'minimum_travel_heading': 0.5,
                'scan_buffer_size': 10,
                'scan_buffer_maximum_scan_distance': 10.0,
                'link_match_minimum_response_fine': 0.1,
                'link_scan_maximum_distance': 15.0,
                'loop_search_max_distance': 10.0,
                'do_loop_closing': True,
                'loop_match_minimum_chain_size': 10,
                'loop_match_maximum_variance_coarse': 3.0,
                'loop_match_minimum_response_coarse': 0.35,
                'loop_match_minimum_response_fine': 0.45,
                'debug_logging': False,
                'throttle_scans': 1,
                'transform_publish_period': 0.02
            }
        }
    }
    
    # SLAM Toolbox node with simulation parameters
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_sim,
            {'use_sim_time': use_sim_time}
        ],
    )
    
    # Create a modified Nav2 params file for simulation
    nav2_params_sim = {
        'amcl': {
            'ros__parameters': {
                'use_sim_time': True,
                'alpha1': 0.2,
                'alpha2': 0.2,
                'alpha3': 0.2,
                'alpha4': 0.2,
                'alpha5': 0.2,
                'base_frame_id': 'base_link',
                'beam_skip_distance': 0.5,
                'beam_skip_threshold': 0.3,
                'do_beamskip': False,
                'global_frame_id': 'map',
                'lambda_short': 0.1,
                'laser_likelihood_max_dist': 2.0,
                'laser_max_range': 100.0,
                'laser_min_range': -1.0,
                'laser_model_type': 'likelihood_field',
                'max_beams': 60,
                'max_particles': 2000,
                'min_particles': 500,
                'odom_frame_id': 'odom',
                'pf_err': 0.05,
                'pf_z': 0.99,
                'recovery_alpha_slow': 0.0,
                'recovery_alpha_fast': 0.0,
                'resample_interval': 1,
                'robot_model_type': 'differential',
                'save_pose_rate': 0.5,
                'sigma_hit': 0.2,
                'tf_broadcast': True,
                'transform_tolerance': 1.0,
                'update_min_a': 0.2,
                'update_min_d': 0.25,
                'z_hit': 0.5,
                'z_max': 0.05,
                'z_rand': 0.5,
                'z_short': 0.05,
                'scan_topic': '/scan'  # Use simulated laser scan
            }
        },
        'bt_navigator': {
            'ros__parameters': {
                'use_sim_time': True,
                'global_frame': 'map',
                'robot_base_frame': 'base_link',
                'odom_topic': '/odom',
                'default_bt_xml_filename': 'navigate_w_replanning_and_recovery.xml'
            }
        },
        'controller_server': {
            'ros__parameters': {
                'use_sim_time': True,
                'controller_frequency': 20.0,
                'min_x_velocity_threshold': 0.001,
                'min_y_velocity_threshold': 0.001,
                'min_theta_velocity_threshold': 0.001
            }
        },
        'local_costmap': {
            'local_costmap': {
                'ros__parameters': {
                    'use_sim_time': True,
                    'update_frequency': 5.0,
                    'publish_frequency': 2.0,
                    'global_frame': 'odom',
                    'robot_base_frame': 'base_link',
                    'rolling_window': True,
                    'width': 3,
                    'height': 3,
                    'resolution': 0.05,
                    'footprint': '[ [0.25, 0.15], [0.25, -0.15], [-0.25, -0.15], [-0.25, 0.15] ]',
                    'plugins': ['obstacle_layer', 'inflation_layer'],
                    'obstacle_layer': {
                        'plugin': 'nav2_costmap_2d::ObstacleLayer',
                        'enabled': True,
                        'observation_sources': 'scan',
                        'scan': {
                            'topic': '/scan',
                            'max_obstacle_height': 2.0,
                            'clearing': True,
                            'marking': True,
                            'data_type': 'LaserScan',
                            'raytrace_max_range': 3.0,
                            'raytrace_min_range': 0.0,
                            'obstacle_max_range': 2.5,
                            'obstacle_min_range': 0.0
                        }
                    },
                    'inflation_layer': {
                        'plugin': 'nav2_costmap_2d::InflationLayer',
                        'cost_scaling_factor': 3.0,
                        'inflation_radius': 0.55
                    },
                    'always_send_full_costmap': True
                }
            }
        },
        'global_costmap': {
            'global_costmap': {
                'ros__parameters': {
                    'use_sim_time': True,
                    'update_frequency': 1.0,
                    'publish_frequency': 1.0,
                    'global_frame': 'map',
                    'robot_base_frame': 'base_link',
                    'robot_radius': 0.3,
                    'resolution': 0.05,
                    'track_unknown_space': True,
                    'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
                    'obstacle_layer': {
                        'plugin': 'nav2_costmap_2d::ObstacleLayer',
                        'enabled': True,
                        'observation_sources': 'scan',
                        'scan': {
                            'topic': '/scan',
                            'max_obstacle_height': 2.0,
                            'clearing': True,
                            'marking': True,
                            'data_type': 'LaserScan',
                            'raytrace_max_range': 3.0,
                            'raytrace_min_range': 0.0,
                            'obstacle_max_range': 2.5,
                            'obstacle_min_range': 0.0
                        }
                    },
                    'static_layer': {
                        'plugin': 'nav2_costmap_2d::StaticLayer',
                        'map_subscribe_transient_local': True
                    },
                    'inflation_layer': {
                        'plugin': 'nav2_costmap_2d::InflationLayer',
                        'cost_scaling_factor': 3.0,
                        'inflation_radius': 0.55
                    },
                    'always_send_full_costmap': True
                }
            }
        }
    }
    
    # Use existing navigation launch file if available
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav_pkg_dir, 'launch', 'navigation.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=LaunchConfigurationEquals('slam', 'false')
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
    
    # Obstacle visualizer node
    obstacle_visualizer_node = Node(
        package='tracked_robot_simulation',
        executable='obstacle_visualizer',
        name='obstacle_visualizer',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'costmap_topic': '/global_costmap/costmap'},
            {'marker_topic': '/obstacle_markers'},
            {'obstacle_threshold': 70},
            {'marker_lifetime': 0.5},
            {'marker_scale': 0.1},
            {'marker_color_r': 1.0},
            {'marker_color_g': 0.0},
            {'marker_color_b': 0.0},
            {'marker_color_a': 0.8}
        ]
    )
    
    # Local costmap obstacle visualizer
    local_obstacle_visualizer_node = Node(
        package='tracked_robot_simulation',
        executable='obstacle_visualizer',
        name='local_obstacle_visualizer',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'costmap_topic': '/local_costmap/costmap'},
            {'marker_topic': '/local_obstacle_markers'},
            {'obstacle_threshold': 70},
            {'marker_lifetime': 0.5},
            {'marker_scale': 0.1},
            {'marker_color_r': 0.0},
            {'marker_color_g': 0.0},
            {'marker_color_b': 1.0},
            {'marker_color_a': 0.8}
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Whether to run SLAM (true) or localization with existing map (false)'),
            
        # Launch components
        gazebo,
        robot_state_publisher_launch,
        spawn_entity,
        bridge,
        simulated_laser_scan,
        slam_toolbox_node,
        nav_launch,
        obstacle_visualizer_node,
        local_obstacle_visualizer_node,
        rviz_node,
    ])
