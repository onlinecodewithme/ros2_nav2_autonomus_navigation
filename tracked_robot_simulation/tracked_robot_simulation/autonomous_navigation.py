#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.duration import Duration
import random
import math
import time

class AutonomousNavigation(Node):
    def __init__(self):
        super().__init__('autonomous_navigation')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.timer = self.create_timer(1.0, self.check_action_server)
        self.navigation_goal_timer = None
        self.get_logger().info('Autonomous navigation node initialized')
        self.action_server_ready = False
        self.current_goal = None
        self.goal_positions = [
            (2.0, 2.0),   # Top right
            (-2.0, 2.0),  # Top left
            (-2.0, -2.0), # Bottom left
            (2.0, -2.0),  # Bottom right
            (0.0, 0.0)    # Center
        ]
        self.current_goal_index = 0

    def check_action_server(self):
        if not self.action_server_ready:
            if self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
                self.action_server_ready = True
                self.get_logger().info('Navigation action server is available!')
                self.timer.cancel()
                self.navigation_goal_timer = self.create_timer(5.0, self.send_navigation_goal)
            else:
                self.get_logger().info('Waiting for navigation action server...')

    def send_navigation_goal(self):
        if self.current_goal is not None:
            # A goal is already being processed
            return

        # Get the next goal position
        x, y = self.goal_positions[self.current_goal_index]
        self.current_goal_index = (self.current_goal_index + 1) % len(self.goal_positions)
        
        # Create the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        # Publish the goal pose for visualization
        self.goal_publisher.publish(goal_pose)
        
        # Create the action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f'Sending navigation goal: x={x}, y={y}')
        
        # Send the goal
        self.current_goal = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self.current_goal.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.current_goal = None
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info(f'Goal failed with status: {status}')

        self.current_goal = None
        # Wait a bit before sending the next goal
        time.sleep(2.0)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}')

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
