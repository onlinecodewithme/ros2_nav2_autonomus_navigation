# Synchronizing Gazebo with RViz

This document explains how the simulation setup synchronizes Gazebo with RViz to ensure that what you see in the Gazebo world is accurately represented in RViz, including costmaps and obstacles.

## Key Components for Synchronization

### 1. ROS-Gazebo Bridge

The `ros_gz_bridge` package is used to bridge messages between ROS 2 and Gazebo. In our launch files, we set up the bridge like this:

```python
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
```

This bridge ensures that:
- Sensor data from Gazebo (laser scans, point clouds, etc.) is published to ROS 2 topics
- Transform information (TF) is shared between Gazebo and ROS 2
- Command velocities from ROS 2 are sent to Gazebo
- Odometry information from Gazebo is published to ROS 2

### 2. Transform Tree (TF)

The transform tree provides the spatial relationships between different coordinate frames. It's crucial for synchronization because it allows RViz to correctly place visualizations in 3D space.

Key components that publish to the TF tree:
- **Robot State Publisher**: Publishes the robot's joint states and transforms from the URDF model
- **Gazebo**: Publishes the robot's pose in the world
- **SLAM Toolbox**: Publishes the map->odom transform

In our setup, we ensure that all these components use the same frame IDs:
- `map`: The global reference frame
- `odom`: The odometry reference frame
- `base_link`: The robot's base reference frame
- `camera_link`: The camera's reference frame

### 3. Sensor Data Processing

Sensor data from Gazebo is processed to create useful visualizations:

1. **Depth Camera to Laser Scan**: We convert depth camera data to laser scan data using the `depthimage_to_laserscan` node:

```python
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
```

2. **SLAM for Mapping**: The SLAM toolbox processes laser scan data to create a map:

```python
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
        'scan_topic': '/scan'
    }],
)
```

3. **Costmap Generation**: The navigation stack processes sensor data to create costmaps, which represent obstacles and free space.

### 4. Obstacle Visualization

Our custom `obstacle_visualizer` node converts costmap data into 3D markers for better visualization in RViz:

```python
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
```

This node:
1. Subscribes to costmap topics
2. Processes the costmap data to identify obstacles
3. Creates 3D markers for the obstacles
4. Publishes the markers to topics that RViz can visualize

### 5. RViz Configuration

Our custom RViz configuration (`full_simulation.rviz`) is set up to display all the synchronized data:

- TF tree
- Robot model
- Map
- Global and local costmaps
- Laser scan data
- Point cloud data
- Path planning
- Obstacle markers

## Ensuring Proper Synchronization

To ensure proper synchronization:

1. **Use Simulation Time**: All nodes use the same time source (simulation time from Gazebo):
   ```python
   {'use_sim_time': True}
   ```

2. **Consistent Frame IDs**: All nodes use the same frame IDs for the robot, map, etc.

3. **Proper Topic Remappings**: Ensure that topics are correctly remapped between Gazebo and ROS 2

4. **Transform Tolerance**: Set appropriate transform tolerance values to handle slight timing discrepancies

## Troubleshooting Synchronization Issues

If you encounter synchronization issues:

1. **Check TF Tree**: Use `ros2 run tf2_tools view_frames` to visualize the TF tree and ensure all frames are connected correctly

2. **Verify Topic Publishing**: Use `ros2 topic list` and `ros2 topic echo` to check if topics are being published correctly

3. **Check Time Synchronization**: Ensure all nodes are using simulation time

4. **Adjust Transform Tolerance**: If transforms are slightly out of sync, try increasing the transform tolerance

5. **Visualize in RViz**: Use RViz to visualize the TF tree, sensor data, and costmaps to identify any issues
