#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import signal
import sys

class AutoTeleop(Node):
    def __init__(self):
        super().__init__('auto_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.get_logger().info('Auto teleop node started')
        self.get_logger().info('Publishing forward motion at 0.2 m/s')
        self.get_logger().info('Press Ctrl+C to stop')
        
        # Set initial speeds
        self.linear_speed = 0.2
        self.angular_speed = 0.0
        
        # Handle Ctrl+C gracefully
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def timer_callback(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher.publish(twist)
        
    def signal_handler(self, sig, frame):
        self.get_logger().info('Stopping robot...')
        # Stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        time.sleep(0.5)  # Give time for the message to be sent
        sys.exit(0)

def main():
    rclpy.init()
    teleop = AutoTeleop()
    
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        teleop.publisher.publish(twist)
        
        # Clean up
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
