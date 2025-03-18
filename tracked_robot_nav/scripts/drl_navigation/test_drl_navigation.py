#!/usr/bin/env python3

"""
Test script for DRL navigation with ZED camera.
This script loads a pre-trained TD3 model and uses it for navigation.
"""

import os
import sys
import rclpy
from rclpy.node import Node
import threading

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drl_navigation.td3_zed_navigation import TD3, OdomSubscriber, ZedSubscriber, GazeboEnv, RealEnv, USE_SIMULATION

def main():
    """
    Main function to run the DRL navigation test.
    """
    rclpy.init(args=None)

    # Set the parameters for the implementation
    seed = 0  # Random seed number
    max_ep = 500  # maximum number of steps per episode
    file_name = "td3_zed"  # name of the file to load the policy from
    environment_dim = 20
    robot_dim = 4

    # Create the network
    state_dim = environment_dim + robot_dim
    action_dim = 2
    network = TD3(state_dim, action_dim)
    
    # Load the pre-trained model
    try:
        network.load(file_name, "./tracked_robot_nav/scripts/drl_navigation/models")
        print("Successfully loaded pre-trained model")
    except Exception as e:
        print(f"Error loading model: {e}")
        print("Using default model parameters")

    # Create the environment and subscribers based on the simulation flag
    if USE_SIMULATION:
        print("Using Gazebo simulation environment")
        env = GazeboEnv()
    else:
        print("Using real hardware environment")
        env = RealEnv()
    
    odom_subscriber = OdomSubscriber()
    zed_subscriber = ZedSubscriber()

    # Set up the executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(env)
    executor.add_node(odom_subscriber)
    executor.add_node(zed_subscriber)

    # Start the executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Create a rate limiter
    rate = env.create_rate(2)

    # Initialize variables
    done = True
    episode_timesteps = 0

    print("Starting DRL navigation test...")
    
    try:
        # Main loop
        while rclpy.ok():
            # On termination of episode
            if done:
                print("Resetting environment...")
                state = env.reset()
                action = network.get_action(state)
                # Update action to fall in range [0,1] for linear velocity and [-1,1] for angular velocity
                a_in = [(action[0] + 1) / 2, action[1]]
                next_state, reward, done, target = env.step(a_in)
                done = 1 if episode_timesteps + 1 == max_ep else int(done)

                done = False
                episode_timesteps = 0
                print("Environment reset complete")
            else:
                # Get action from the network
                action = network.get_action(state)
                # Update action to fall in range [0,1] for linear velocity and [-1,1] for angular velocity
                a_in = [(action[0] + 1) / 2, action[1]]
                
                # Take a step in the environment
                next_state, reward, done, target = env.step(a_in)
                done = 1 if episode_timesteps + 1 == max_ep else int(done)

                # Update state and timesteps
                state = next_state
                episode_timesteps += 1
                
                # Print status every 10 steps
                if episode_timesteps % 10 == 0:
                    print(f"Step: {episode_timesteps}, Action: {a_in}, Reward: {reward}")
                
                # If target reached, print success message
                if target:
                    print("Target reached! Resetting environment...")
    
    except KeyboardInterrupt:
        print("Navigation stopped by user")
    except Exception as e:
        print(f"Error during navigation: {e}")
    finally:
        # Clean up
        rclpy.shutdown()
        print("Navigation test complete")

if __name__ == "__main__":
    main()
