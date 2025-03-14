# Tracked Robot Simulation

This package provides a simulation environment for the tracked robot to test navigation in unknown environments using Gazebo and ROS 2.

## Overview

The simulation setup allows you to:

1. Simulate the tracked robot in a Gazebo environment
2. Test SLAM and mapping capabilities in unknown environments
3. Test autonomous navigation and exploration
4. Validate robot control algorithms before deploying to real hardware

## Prerequisites

- ROS 2 (tested with Humble)
- Gazebo Garden (via ros-gz)
- Nav2 packages
- SLAM Toolbox
- explore_lite package (for autonomous exploration)
- depthimage_to_laserscan package

Install the required packages:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros-gz \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-depthimage-to-laserscan
```

For the explore_lite package:

```bash
cd ~/ros2_ws/src
git clone https://github.com/robo-friends/m-explore-ros2.git
cd ..
colcon build --symlink-install --packages-select explore_lite
```

## Building the Simulation

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select tracked_robot_simulation
source install/setup.bash
```

## Running the Simulation

### Basic Gazebo Simulation

To launch just the Gazebo simulation with the robot:

```bash
ros2 launch tracked_robot_simulation gazebo_simulation.launch.py
```

### Navigation in Unknown Environment

To launch the full navigation stack with SLAM and autonomous exploration:

```bash
ros2 launch tracked_robot_simulation navigation_simulation.launch.py
```

Launch arguments:
- `slam:=true|false` - Whether to run SLAM (true) or localization with existing map (false)
- `map:=path/to/map.yaml` - Path to map file when running in localization mode
- `world_file:=world_name.world` - Name of the world file to load (default: unknown_environment.world)

### Using a Pre-built Map

If you've already created a map and want to use it for navigation:

```bash
ros2 launch tracked_robot_simulation navigation_simulation.launch.py slam:=false map:=/path/to/your/map.yaml
```

## Available Worlds

The package includes the following simulation worlds:

1. `unknown_environment.world` - A maze-like environment with walls and obstacles for testing navigation

## Customizing the Simulation

### Creating New Worlds

You can create new Gazebo worlds and place them in the `worlds` directory. Then use the `world_file` launch argument to specify your custom world:

```bash
ros2 launch tracked_robot_simulation navigation_simulation.launch.py world_file:=your_custom_world.world
```

### Modifying Robot Parameters

The simulation uses the same URDF model as the real robot. To modify robot parameters, edit the files in the `tracked_robot_description` package.

### Tuning Navigation Parameters

Navigation parameters can be adjusted in the `tracked_robot_nav/config/nav2_params.yaml` file.

## Testing and Validation

### Manual Control

You can manually control the robot in simulation using the teleop keyboard:

```bash
ros2 launch tracked_robot_simulation teleop.launch.py
```

This will open a terminal window with keyboard controls:
- Use the arrow keys to move the robot
- Use 'q', 'w', 'e', 'a', 's', 'd', 'z', 'x', 'c' for more precise control
- Press 'k' to stop the robot
- Use 'i' and 'o' to increase/decrease speed

Alternatively, you can manually publish to the `/cmd_vel` topic:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Setting Navigation Goals

In RViz, use the "2D Goal Pose" tool to set navigation goals for the robot.

### Saving Maps

When using SLAM, you can save the generated map:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_simulated_map'}}"
```

## Autonomous Exploration

The simulation includes the explore_lite package for autonomous exploration of unknown environments. The robot will automatically explore the environment and build a map.

You can monitor the exploration progress in RViz, where frontier points are displayed as markers.

## Transferring to Real Robot

After validating your navigation setup in simulation, you can transfer the same parameters and configurations to the real robot. The simulation is designed to match the real robot's behavior as closely as possible.

Key differences to be aware of:
- Sensor noise and physical interactions will differ between simulation and reality
- Motor response may require additional tuning on the real hardware
- Environmental factors like lighting and surface texture can affect camera-based perception

## Troubleshooting

### Gazebo Not Launching

If Gazebo fails to launch, try resetting the Gazebo cache:

```bash
rm -rf ~/.gazebo/
```

### Navigation Issues

If the robot is not navigating correctly in simulation:

1. Check the transform tree in RViz to ensure all frames are properly connected
2. Verify that the simulated laser scan is being published correctly
3. Adjust the navigation parameters in nav2_params.yaml

### Visualization Issues

If RViz is not displaying certain elements:

1. Ensure all topics are being published by checking with `ros2 topic list`
2. Reset the RViz configuration or use the provided simulation.rviz file
