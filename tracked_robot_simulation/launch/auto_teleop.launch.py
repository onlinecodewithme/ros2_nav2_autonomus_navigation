import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    pkg_dir = get_package_share_directory('tracked_robot_simulation')
    teleop_script = os.path.join(pkg_dir, 'scripts', 'auto_teleop.py')
    
    # Auto teleop node
    teleop_node = Node(
        executable=teleop_script,
        name='auto_teleop',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        teleop_node
    ])
