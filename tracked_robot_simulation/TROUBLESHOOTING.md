# Troubleshooting Guide for Simulation Visualization

If you're having issues with the visualization in RViz, this guide will help you diagnose and fix common problems.

## Common Issues and Solutions

### 1. Robot Model Not Visible in RViz

**Symptoms:**
- RViz is running but the robot model is not visible
- TF frames may be visible but not the actual robot model

**Possible Causes and Solutions:**

a) **Robot Description Not Published:**
   - Check if the robot description is being published:
     ```bash
     ros2 topic echo /robot_description --once
     ```
   - If not published, ensure the robot_state_publisher is running:
     ```bash
     ros2 node list | grep robot_state_publisher
     ```

b) **TF Tree Issues:**
   - Check if the TF tree is properly connected:
     ```bash
     ros2 run tf2_tools view_frames
     ```
   - Ensure there's a path from the fixed frame (usually "map") to the robot's base_link

c) **Fixed Frame Setting:**
   - In RViz, check if the "Fixed Frame" in Global Options is set to a frame that exists (e.g., "map" or "odom")
   - Try changing the fixed frame to "base_link" temporarily to see if the robot appears

d) **URDF/SDF Issues:**
   - Verify that the URDF/SDF file is correctly formatted and all meshes/geometries are properly defined
   - Check for any error messages related to the robot model in the terminal

### 2. Costmaps Not Visible in RViz

**Symptoms:**
- No costmap displays in RViz despite being configured to show them
- Costmap topics may not be available

**Possible Causes and Solutions:**

a) **Navigation Stack Not Running:**
   - Check if the navigation nodes are running:
     ```bash
     ros2 node list | grep -E 'costmap|planner|controller'
     ```
   - If not running, ensure the navigation launch file is included in your main launch file

b) **Costmap Topics Not Published:**
   - Check if costmap topics are being published:
     ```bash
     ros2 topic list | grep costmap
     ros2 topic echo /global_costmap/costmap --once
     ```
   - If not published, check the navigation stack configuration

c) **RViz Configuration:**
   - Ensure the Map displays in RViz are properly configured with the correct topics
   - Try adding a new Map display and setting it to the costmap topic manually

d) **Transform Issues:**
   - Costmaps require proper transforms between frames
   - Check if all required transforms are available:
     ```bash
     ros2 run tf2_tools view_frames
     ```

### 3. Gazebo and RViz Not Synchronized

**Symptoms:**
- Robot position in Gazebo doesn't match position in RViz
- Sensor data in RViz doesn't align with the Gazebo world

**Possible Causes and Solutions:**

a) **Bridge Issues:**
   - Ensure the ROS-Gazebo bridge is running and properly configured:
     ```bash
     ros2 node list | grep bridge
     ```
   - Check if bridge topics are being published:
     ```bash
     ros2 topic list | grep gz
     ```

b) **Transform Tree Disconnected:**
   - The transform tree should connect all frames from Gazebo to RViz
   - Run the TF viewer to check connections:
     ```bash
     ros2 run tf2_tools view_frames
     ```
   - Add static transform publishers if needed to connect frames

c) **Time Synchronization:**
   - Ensure all nodes are using the same time source:
     ```bash
     ros2 param list | grep use_sim_time
     ```
   - All nodes should have use_sim_time=true for simulation

d) **Topic Remappings:**
   - Check if topics are correctly remapped between Gazebo and ROS 2:
     ```bash
     ros2 topic list
     ```
   - Ensure the bridge is configured to bridge all necessary topics

### 4. Debugging with the Debug Launch File

We've provided a special debug launch file to help diagnose visualization issues:

```bash
ros2 launch tracked_robot_simulation debug_visualization.launch.py
```

This launch file:
- Starts Gazebo with the robot
- Sets up the ROS-Gazebo bridge
- Publishes necessary static transforms
- Starts RViz with a simplified configuration
- Runs diagnostic tools like tf_echo and topic listing

Watch the console output for any error messages and check the TF tree visualization to ensure all frames are connected properly.

### 5. Checking Specific Components

a) **Robot State Publisher:**
   ```bash
   ros2 topic echo /robot_description --once
   ros2 topic echo /tf --once
   ```

b) **Gazebo-ROS Bridge:**
   ```bash
   ros2 topic list | grep -E 'scan|camera|odom'
   ```

c) **Navigation Stack:**
   ```bash
   ros2 topic list | grep -E 'costmap|plan|footprint'
   ```

d) **SLAM:**
   ```bash
   ros2 topic list | grep map
   ros2 topic echo /map --once
   ```

### 6. Advanced Debugging

If the above steps don't resolve your issue:

1. **Enable Verbose Logging:**
   ```bash
   export RCUTILS_LOGGING_LEVEL=DEBUG
   ```

2. **Check ROS 2 Daemon:**
   ```bash
   ros2 daemon status
   ros2 daemon stop
   ros2 daemon start
   ```

3. **Inspect Node Connections:**
   ```bash
   ros2 node info /[node_name]
   ```

4. **Check for Resource Conflicts:**
   - Ensure no other Gazebo or RViz instances are running
   - Check system resources (CPU, memory) to ensure they're not exhausted

5. **Rebuild with Debug Symbols:**
   ```bash
   colcon build --packages-select tracked_robot_simulation --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   ```

## Common Error Messages and Solutions

### "Transform from X to Y not available"
- Missing transform in the TF tree
- Add a static transform publisher or ensure the dynamic transform is being published

### "No transform from [source_frame] to [target_frame]"
- Similar to above, indicates a break in the TF tree
- Check if all required frames are being published with:
  ```bash
  ros2 run tf2_tools view_frames
  ```

### "Could not find a connection between 'map' and 'base_link' because they are not part of the same tree"
- This indicates that the TF tree is disconnected between the map and robot frames
- The proper TF tree for navigation should be: map → odom → base_link
- Solutions:
  1. Check if the odom_to_tf node is running and properly configured:
     ```bash
     ros2 node list | grep odom_to_tf
     ```
  2. Ensure the odom_to_tf node is using the correct parent_frame parameter:
     ```bash
     ros2 param get /odom_to_tf_py parent_frame
     ```
     It should be set to "odom", not "map"
  3. Verify that a transform from map to odom is being published:
     ```bash
     ros2 run tf2_ros tf2_echo map odom
     ```
  4. If using the Python version, ensure it's publishing both:
     - A static transform from map to odom
     - A dynamic transform from odom to base_link based on odometry data

### "Failed to load plugin..."
- Missing ROS 2 package or plugin
- Install the required package:
  ```bash
  sudo apt install ros-humble-[package-name]
  ```

### "Could not find node: [node_name]"
- Node failed to start or crashed
- Check the console output for error messages from that node
- Ensure all dependencies are installed

## Getting Help

If you're still having issues after trying these troubleshooting steps, you can:

1. Check the ROS 2 Answers forum: https://answers.ros.org/
2. Look for similar issues on GitHub: https://github.com/ros-planning/navigation2/issues
3. Ask for help on the ROS Discourse: https://discourse.ros.org/
