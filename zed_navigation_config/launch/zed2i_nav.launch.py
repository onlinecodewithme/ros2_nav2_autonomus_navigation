import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    sim_mode = LaunchConfiguration('sim_mode', default='false')
    
    # Camera model
    camera_model = 'zed2i'
    
    # Log info for simulation mode
    sim_info = LogInfo(
        condition=IfCondition(sim_mode),
        msg="Running in simulation mode - ZED camera hardware not used"
    )
    
    # Include the ZED camera launch file
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_camera_launch = IncludeLaunchDescription(
        condition=UnlessCondition(sim_mode),
        launch_description_source=PythonLaunchDescriptionSource([
            zed_wrapper_dir, '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': camera_model,
            'camera_name': camera_model,
            'publish_tf': 'true',
            'publish_map_tf': 'true',
            'sim_mode': 'false'
        }.items()
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),
        
        DeclareLaunchArgument(
            'sim_mode',
            default_value='false',
            description='Run in simulation mode'),
            
        # Launch components
        sim_info,
        zed_camera_launch
    ])
