# DRL Navigation Quick Start Guide

This guide provides step-by-step instructions to quickly get started with the Deep Reinforcement Learning (DRL) navigation system in the tracked_robot_nav project.

## Prerequisites

- ROS2 (Humble or later) installed and configured
- ZED SDK installed
- CUDA-compatible GPU (recommended for training)
- Python 3.8 or later with the following packages:
  - PyTorch
  - NumPy
  - ROS2 Python client library

## Installation

1. Clone the repository (if not already done):
   ```bash
   git clone https://github.com/your-organization/tracked_robot_nav.git
   cd tracked_robot_nav
   ```

2. Install Python dependencies:
   ```bash
   pip install torch numpy matplotlib
   ```

3. Build the ROS package:
   ```bash
   cd /path/to/ros_workspace
   colcon build --packages-select tracked_robot_nav
   source install/setup.bash
   ```

## Quick Start: Testing Pre-trained Model

1. Connect and configure your ZED camera:
   ```bash
   # Check if ZED camera is detected
   lsusb | grep Stereolabs
   ```

2. Launch the ZED camera node:
   ```bash
   ros2 launch zed_wrapper zed2.launch.py
   ```

3. Run the DRL navigation with a pre-trained model:
   ```bash
   cd /home/x4/x4_autonomus/src/tracked_robot_nav
   ./scripts/start_drl_navigation.sh --test --model_path models/td3_navigation_model.pt
   ```

4. Set a navigation goal using RViz or the command line:
   ```bash
   # Using command line
   ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}" --once
   ```

## Quick Start: Training a New Model

1. Launch the simulation environment (recommended for initial training):
   ```bash
   ros2 launch tracked_robot_simulation gazebo_world.launch.py world:=empty.world
   ```

2. Start the training process:
   ```bash
   cd /home/x4/x4_autonomus/src/tracked_robot_nav
   ./scripts/start_drl_navigation.sh --train --save_path models/my_new_model.pt
   ```

3. Monitor the training progress:
   ```bash
   # In a new terminal
   cd /home/x4/x4_autonomus/src/tracked_robot_nav/logs
   tail -f drl_training_latest.log
   ```

4. Visualize training in RViz (optional):
   ```bash
   ros2 launch tracked_robot_nav rviz_drl_training.launch.py
   ```

## Key Parameters

You can customize the DRL navigation behavior by modifying the following parameters in the `config/drl_navigation_params.yaml` file:

- `learning_rate`: Controls how quickly the model adapts (default: 0.0003)
- `gamma`: Discount factor for future rewards (default: 0.99)
- `batch_size`: Number of experiences used for each update (default: 256)
- `buffer_size`: Size of the replay buffer (default: 1000000)
- `exploration_noise`: Standard deviation of exploration noise (default: 0.1)

## Common Commands

- Start training with custom parameters:
  ```bash
  ./scripts/start_drl_navigation.sh --train --lr 0.0001 --gamma 0.98 --noise 0.2
  ```

- Resume training from an existing model:
  ```bash
  ./scripts/start_drl_navigation.sh --train --model_path models/previous_model.pt --save_path models/improved_model.pt
  ```

- Test with visualization:
  ```bash
  ./scripts/start_drl_navigation.sh --test --model_path models/my_model.pt --visualize
  ```

- Run with ROS2 navigation integration:
  ```bash
  ./scripts/start_drl_navigation.sh --test --model_path models/my_model.pt --ros_nav_integration
  ```

## Troubleshooting

- **Issue**: ZED camera not detected
  **Solution**: Check USB connection and run `zed_diagnostic` tool

- **Issue**: Training is unstable
  **Solution**: Reduce learning rate and increase batch size

- **Issue**: Robot not moving during testing
  **Solution**: Check action scaling in the configuration file

- **Issue**: High memory usage
  **Solution**: Reduce replay buffer size and batch size

## Next Steps

- Read the full [DRL Navigation README](./DRL_NAVIGATION_README.md) for detailed information
- Explore the code in `scripts/drl_navigation/` to understand the implementation
- Try training in different environments to improve generalization
- Experiment with different reward functions by modifying `td3_zed_navigation.py`
