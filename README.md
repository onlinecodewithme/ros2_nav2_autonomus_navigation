# Tracked Robot ROS 2 Project

This repository contains a complete ROS 2 implementation for a tracked robot with navigation, SLAM, and simulation capabilities. The project integrates a ZED2i camera for perception and supports both real hardware operation and simulation testing.

## Project Overview

This project provides a complete software stack for a tracked robot platform, including:

- Robot description (URDF)
- Motor control and odometry
- Navigation and SLAM
- ZED camera integration
- Gazebo simulation environment
- Autonomous exploration capabilities

The system is designed to work with a tracked robot platform equipped with a ZED2i stereo camera for perception. The robot can be controlled manually or autonomously navigate through unknown environments using SLAM for mapping and localization.

### Hardware Components

- Tracked robot base with differential drive control
- ZED2i stereo camera for depth perception and visual odometry
- Jetson Orin NX for onboard processing (for real hardware)
- Motor controllers for track actuation

### Software Architecture

The software is built on ROS 2 Humble and follows a modular design:

1. **Perception**: ZED camera provides RGB images, depth data, and visual odometry
2. **Mapping**: SLAM Toolbox creates and maintains maps of the environment
3. **Navigation**: Nav2 stack handles path planning and obstacle avoidance
4. **Control**: Custom controllers translate velocity commands to track movements
5. **Simulation**: Ignition Gazebo provides a realistic testing environment

## Package Structure

The project is organized into the following packages:

### Core Packages

- **tracked_robot_description**: URDF model and robot state publisher
- **tracked_robot_controller**: Motor control and odometry nodes
- **tracked_robot_nav**: Navigation, mapping, and SLAM configurations
- **tracked_robot_simulation**: Gazebo simulation environment
- **zed_navigation_config**: ZED camera configuration for navigation

### ZED Camera Integration

- **zed-ros2-wrapper**: Core ZED camera drivers and interfaces
- **zed-ros2-examples**: Example applications using ZED cameras

### Exploration

- **m-explore-ros2**: Autonomous exploration and map merging

## Installation

### Prerequisites

- ROS 2 Humble
- Ignition Gazebo
- ZED SDK (for hardware operation)
- Nav2 packages
- SLAM Toolbox

### Dependencies Installation

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-ros-gz \
  ros-humble-depthimage-to-laserscan
```

For ZED SDK installation, follow the instructions at: https://www.stereolabs.com/docs/installation/

### Building the Project

```bash
# Clone the repository (if not already done)
cd ~/ros2_ws/src
git clone <repository-url>

# Build the packages
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Hardware Operation

To launch the complete robot stack on real hardware:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch tracked_robot_nav robot_complete.launch.py
```

#### Mapping with SLAM

```bash
ros2 launch tracked_robot_nav mapping.launch.py
```

For mapping with ZED camera:

```bash
ros2 launch tracked_robot_nav mapping_zed.launch.py
```

#### Navigation with Existing Map

```bash
ros2 launch tracked_robot_nav navigation.launch.py map:=/path/to/map.yaml
```

### Simulation

#### Basic Simulation

```bash
ros2 launch tracked_robot_simulation gazebo_simulation.launch.py
```

#### Navigation Simulation with SLAM

```bash
ros2 launch tracked_robot_simulation navigation_simulation.launch.py
```

#### Manual Control

There are several ways to control the robot manually:

1. **Auto Teleop (Recommended for Testing)**:
   ```bash
   ros2 launch tracked_robot_simulation auto_teleop.launch.py
   ```
   This launches a non-interactive teleop node that automatically sends forward motion commands at 0.2 m/s.
   It doesn't require any keyboard input, making it ideal for testing in any environment.
   Press Ctrl+C to stop the robot.

2. **Simple Teleop**:
   ```bash
   ros2 launch tracked_robot_simulation simple_teleop.launch.py
   ```
   This launches a custom teleop script that works in interactive terminal environments.
   
   Use the keyboard to control the robot:
   - 'w'/'x': increase/decrease linear velocity
   - 'a'/'d': increase/decrease angular velocity
   - 's': stop
   - 'q'/'e'/'z'/'c': diagonal movement
   - CTRL-C to quit
   
   **Note**: This may still encounter the "Inappropriate ioctl for device" error in some environments.

3. **Standard ROS 2 Teleop**:
   ```bash
   ros2 launch tracked_robot_simulation teleop.launch.py
   ```
   
   Use the keyboard to control the robot:
   - Arrow keys for basic movement
   - 'k' to stop the robot
   - 'q', 'w', 'e', 'a', 's', 'd', 'z', 'x', 'c' for more precise control
   - 'i' and 'o' to increase/decrease speed
   
   **Note**: The teleop_twist_keyboard node may encounter issues in some terminal environments due to the "Inappropriate ioctl for device" error.

4. **Direct Command Publishing**:
   ```bash
   # Single command
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   
   # Continuous commands (10 Hz)
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```

## Configuration

### Robot Parameters

The tracked robot's physical parameters can be configured in:
- `tracked_robot_description/urdf/tracked_robot.urdf`: Robot model
- `tracked_robot_controller/src/cmd_vel_to_tracks.cpp`: Track control parameters
- `tracked_robot_controller/src/track_odometry.cpp`: Odometry parameters

### Navigation Parameters

Navigation parameters are defined in:
- `tracked_robot_nav/config/nav2_params.yaml`: Navigation stack parameters
- `tracked_robot_nav/config/slam_params.yaml`: SLAM parameters

### ZED Camera Configuration

ZED camera parameters can be configured in:
- `zed_navigation_config/launch/zed2i_nav.launch.py`: Camera parameters
- `zed-ros2-wrapper/zed_wrapper/config/`: Camera configuration files

### Simulation Environment

The simulation environment can be modified in:
- `tracked_robot_simulation/worlds/unknown_environment.world`: Gazebo world file
- `tracked_robot_simulation/config/simulation.rviz`: RViz configuration

## Saving and Loading Maps

To save a map created with SLAM:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"
```

The map will be saved in the current directory. To load a map for navigation:

```bash
ros2 launch tracked_robot_nav navigation.launch.py map:=/path/to/my_map.yaml
```

## Troubleshooting

### Common Issues

#### ZED Camera Not Detected

Ensure the ZED SDK is properly installed and the camera is connected:

```bash
# Check if the camera is detected
lsusb | grep Stereo
```

#### Navigation Issues

If the robot is not navigating correctly:

1. Check the transform tree in RViz to ensure all frames are properly connected
2. Verify that the laser scan or point cloud data is being published
3. Adjust the navigation parameters in `nav2_params.yaml`

#### Simulation Issues

If the simulation is not working correctly:

1. Check if Gazebo is properly installed and running
2. Verify that the robot model is correctly spawned in the simulation
3. Check the ROS-Gazebo bridge for proper topic connections

##### Current Known Issues

- **Teleop Keyboard Issues**: The teleop_twist_keyboard may fail with "Inappropriate ioctl for device" errors in some environments. Use direct topic publishing as a workaround.
  
- **Gazebo Model Loading**: If Gazebo fails to load models with errors like "Unable to find uri[model://sun]", this indicates missing model paths. The project has been updated to use direct model definitions in the world files to avoid this issue.

- **ZED Camera in Simulation**: The simulation uses a simulated depth camera instead of the actual ZED camera driver. The zed_navigation_config package provides a conditional launch system that detects simulation mode and skips launching the actual ZED camera nodes.

- **Navigation in Simulation**: If the robot doesn't move in simulation, try publishing velocity commands directly:
  ```bash
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```
  This will send a continuous command at 10Hz to move the robot forward at 0.2 m/s.

## Contributing

Contributions to this project are welcome. Please follow these steps:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This project is licensed under the Apache 2.0 License - see the LICENSE file for details.
