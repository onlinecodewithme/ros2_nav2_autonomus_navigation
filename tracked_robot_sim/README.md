# Tracked Robot Simulation

This package provides a simulation of a tracked robot using Ignition Gazebo (Gazebo Garden).

## Prerequisites

- ROS 2 Humble
- Ignition Gazebo (Gazebo Garden)
- ros_gz packages for ROS 2 and Gazebo integration

## Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select tracked_robot_sim
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Running the Simulation

### Basic Simulation

To run the basic simulation without RViz (recommended for systems with OpenGL issues):

```bash
ros2 launch tracked_robot_sim tracked_robot.launch.py
```

### With RViz Visualization

To run the simulation with RViz visualization (may crash on some systems):

```bash
ros2 launch tracked_robot_sim tracked_robot.launch.py use_rviz:=true
```

### With ZED Camera

To enable the ZED 2i camera:

```bash
ros2 launch tracked_robot_sim tracked_robot.launch.py use_zed:=true
```

### With Navigation and Costmap

To run the simulation with navigation stack and costmap visualization in RViz:

```bash
ros2 launch tracked_robot_sim tracked_robot_with_nav.launch.py
```

This launch file includes:
- SLAM Toolbox for mapping
- Local and global costmaps
- Point cloud to laser scan conversion
- Navigation visualization in RViz

You can disable specific components:
```bash
# Disable RViz (if it crashes)
ros2 launch tracked_robot_sim tracked_robot_with_nav.launch.py use_rviz:=false

# Disable navigation stack
ros2 launch tracked_robot_sim tracked_robot_with_nav.launch.py use_nav:=false

# Enable ZED camera
ros2 launch tracked_robot_sim tracked_robot_with_nav.launch.py use_zed:=true
```

### Launching RViz Separately

If RViz crashes when launched with the simulation, you can run it separately:

```bash
# First, run the simulation without RViz
ros2 launch tracked_robot_sim tracked_robot_with_nav.launch.py use_rviz:=false

# Then, in a separate terminal, launch RViz with the costmap configuration
ros2 launch tracked_robot_sim rviz_costmap.launch.py
```

If you encounter QoS compatibility issues with the scan topic, use the QoS-compatible RViz launch:

```bash
ros2 launch tracked_robot_sim rviz_with_qos.launch.py
```

This allows you to restart RViz independently if it crashes, without having to restart the entire simulation.

### Testing Gazebo Only

If you want to test just the Gazebo simulation with the robot model:

```bash
ros2 launch tracked_robot_sim gazebo_only.launch.py
```

This launches Gazebo with the world and spawns the robot model, without any ROS nodes or RViz.

## Controlling the Robot

The robot can be controlled using the teleop_twist_keyboard package:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/tracked_robot/cmd_vel
```

### Control Modes

The simulation supports different control modes:

- **Keyboard** (default): Control using teleop_twist_keyboard
- **Joystick**: Control using a joystick/gamepad

To use joystick control:

```bash
ros2 launch tracked_robot_sim tracked_robot.launch.py control_mode:=joystick
```

## Troubleshooting

### OpenGL/RViz Issues

If you encounter OpenGL errors or RViz crashes, you have two options:

1. Run the simulation without RViz:
```bash
ros2 launch tracked_robot_sim tracked_robot.launch.py use_rviz:=false
```

2. Launch RViz separately in a different terminal:
```bash
# First, run the simulation without RViz
ros2 launch tracked_robot_sim tracked_robot_with_nav.launch.py use_rviz:=false

# Then, in a separate terminal, launch RViz
ros2 launch tracked_robot_sim rviz_costmap.launch.py
```

This way, if RViz crashes, it won't affect the running simulation.

### Gazebo Not Finding World Files

If Gazebo cannot find the world files, make sure the package is properly built and installed:

```bash
cd ~/ros2_ws
colcon build --packages-select tracked_robot_sim
source ~/ros2_ws/install/setup.bash
```

### Missing Mesh Files

Some mesh files referenced in the world file might be missing. The simulation has been configured to use simple geometric shapes as replacements.

## License

[License information]
