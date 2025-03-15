# Full Simulation Setup for Tracked Robot

This package provides a complete simulation environment for the tracked robot using Gazebo and RViz. The simulation includes:

- Gazebo simulation with a custom world containing obstacles
- Navigation stack with costmaps
- SLAM for mapping the environment
- Visualization of costmaps and obstacles in RViz
- Synchronization between Gazebo and RViz (see [GAZEBO_RVIZ_SYNC.md](GAZEBO_RVIZ_SYNC.md) for details)

## Prerequisites

Make sure you have the following packages installed:

```bash
sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-depthimage-to-laserscan ros-humble-ros-gz
```

## Building

Build the package using colcon:

```bash
cd ~/ros2_ws
colcon build --packages-select tracked_robot_simulation
source install/setup.bash
```

## Running the Simulation

To run the full simulation:

```bash
ros2 launch tracked_robot_simulation full_simulation.launch.py
```

This will start:
1. Gazebo with the unknown environment world
2. The robot with simulated sensors
3. SLAM for mapping
4. Navigation stack with costmaps
5. RViz with a custom configuration showing all the relevant information

## Components

### 1. Gazebo Simulation

The simulation uses Gazebo Garden (Ignition Gazebo) with a custom world defined in `worlds/unknown_environment.world`. The robot model is defined in `models/tracked_robot.sdf` and includes:

- A differential drive base with tracks
- A depth camera for sensing
- IMU sensor

### 2. ROS 2 - Gazebo Bridge

The simulation uses `ros_gz_bridge` to connect ROS 2 and Gazebo, bridging the following topics:

- `/cmd_vel` - Robot velocity commands
- `/odom` - Odometry information
- `/tf` - Transform information
- `/scan` - Laser scan data (simulated from depth camera)
- `/camera/depth/image_raw` - Depth camera image
- `/camera/depth/camera_info` - Camera information
- `/camera/depth/points` - Point cloud data

### 3. Navigation and SLAM

The simulation includes:

- SLAM Toolbox for mapping
- Nav2 for navigation
- Costmap visualization
- Path planning visualization

### 4. Obstacle Visualization

The `obstacle_visualizer` node converts costmap information into 3D markers for better visualization in RViz. It shows:

- Global costmap obstacles in red
- Local costmap obstacles in blue

## Customization

### Modifying the World

You can modify the Gazebo world by editing `worlds/unknown_environment.world`. Add or remove obstacles as needed.

### Changing Robot Parameters

Robot parameters can be modified in:
- `models/tracked_robot.sdf` - For physical properties and sensors
- `tracked_robot_description/urdf/tracked_robot.urdf` - For the URDF model

### Adjusting Navigation Parameters

Navigation parameters can be adjusted in:
- `tracked_robot_nav/config/nav2_params.yaml` - For Nav2 parameters
- `tracked_robot_nav/config/slam_params.yaml` - For SLAM parameters

## Troubleshooting

### Common Issues

1. **Gazebo doesn't start**: Make sure you have Gazebo Garden installed and the `IGN_GAZEBO_RESOURCE_PATH` environment variable is set correctly.

2. **Robot doesn't appear in Gazebo**: Check that the robot model is correctly defined and the spawn entity command is working.

3. **No costmap in RViz**: Ensure that the navigation stack is running and publishing costmap data.

4. **No synchronization between Gazebo and RViz**: Check that the TF tree is correctly set up and the bridge is working. See [GAZEBO_RVIZ_SYNC.md](GAZEBO_RVIZ_SYNC.md) for detailed information on how the synchronization works and how to troubleshoot issues.

### Logs

Check the terminal output for error messages. You can also use `ros2 topic echo` to check if topics are being published correctly.

## Additional Documentation

- [GAZEBO_RVIZ_SYNC.md](GAZEBO_RVIZ_SYNC.md): Detailed explanation of how Gazebo and RViz are synchronized, including the role of the ROS-Gazebo bridge, transform tree, sensor data processing, and obstacle visualization.
