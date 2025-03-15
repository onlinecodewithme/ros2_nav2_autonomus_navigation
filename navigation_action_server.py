#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import time
import threading

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        
        # Use ReentrantCallbackGroup to allow concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Create the action server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group)
        
        # Current robot pose (simulated)
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        
        # Status publisher (for visualization)
        self.robot_pose_publisher = self.create_publisher(
            PoseStamped, 'robot_pose', 10)
        
        # Timer to publish current robot pose
        self.create_timer(0.1, self.publish_robot_pose, callback_group=self.callback_group)
        
        self.get_logger().info('Navigation Action Server has been started')
    
    def publish_robot_pose(self):
        """Publish the current robot pose for visualization"""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = self.current_pose['x']
        pose_msg.pose.position.y = self.current_pose['y']
        pose_msg.pose.position.z = 0.0
        
        # Convert theta to quaternion (simplified, only yaw)
        pose_msg.pose.orientation.w = math.cos(self.current_pose['theta'] / 2.0)
        pose_msg.pose.orientation.z = math.sin(self.current_pose['theta'] / 2.0)
        
        self.robot_pose_publisher.publish(pose_msg)
    
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action"""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        """Execute the goal"""
        self.get_logger().info('Executing goal...')
        
        # Get the target pose
        goal_pose = goal_handle.request.pose
        target_x = goal_pose.pose.position.x
        target_y = goal_pose.pose.position.y
        
        # Create a result message
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()
        
        # Get starting position
        start_x = self.current_pose['x']
        start_y = self.current_pose['y']
        
        # Calculate total distance
        total_distance = math.sqrt((target_x - start_x)**2 + (target_y - start_y)**2)
        initial_distance = total_distance
        
        # Simulate robot movement
        # We'll update at about 10Hz
        update_rate = 0.1  # seconds
        move_speed = 0.5   # meters per second
        
        # Continue until we reach the goal
        while total_distance > 0.1:  # 10cm tolerance
            # Check if the goal has been canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return NavigateToPose.Result()
            
            # Calculate movement for this step
            step_distance = move_speed * update_rate
            if step_distance > total_distance:
                step_distance = total_distance
            
            # Calculate direction vector
            direction_x = (target_x - self.current_pose['x']) / total_distance
            direction_y = (target_y - self.current_pose['y']) / total_distance
            
            # Update robot position
            self.current_pose['x'] += direction_x * step_distance
            self.current_pose['y'] += direction_y * step_distance
            
            # Update robot orientation (face the direction of travel)
            self.current_pose['theta'] = math.atan2(direction_y, direction_x)
            
            # Recalculate remaining distance
            total_distance = math.sqrt(
                (target_x - self.current_pose['x'])**2 + 
                (target_y - self.current_pose['y'])**2
            )
            
            # Publish feedback
            feedback_msg.distance_remaining = total_distance
            feedback_msg.estimated_time_remaining = rclpy.duration.Duration(
                seconds=total_distance / move_speed
            ).to_msg()
            feedback_msg.navigation_time = rclpy.duration.Duration(
                seconds=(initial_distance - total_distance) / move_speed
            ).to_msg()
            feedback_msg.number_of_recoveries = 0
            feedback_msg.current_pose = goal_pose  # Use the same format as goal
            feedback_msg.current_pose.pose.position.x = self.current_pose['x']
            feedback_msg.current_pose.pose.position.y = self.current_pose['y']
            
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Distance remaining: {total_distance:.2f}')
            
            # Sleep to simulate robot movement time
            time.sleep(update_rate)
        
        # Set the final pose to exactly the target
        self.current_pose['x'] = target_x
        self.current_pose['y'] = target_y
        
        # Goal succeeded
        goal_handle.succeed()
        self.get_logger().info('Goal succeeded! Reached the target pose.')
        
        return result

def main(args=None):
    rclpy.init(args=args)
    
    # Create the action server node
    node = NavigationActionServer()
    
    # Use a multi-threaded executor to handle callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # Keep the main thread alive
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()