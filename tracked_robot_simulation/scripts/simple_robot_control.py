#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import threading
import time

msg = """
Control Your Robot!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
s : stop
q/e/z/c : diagonal movement

CTRL-C to quit
"""

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        self.get_logger().info('Keyboard control node initialized')
        self.get_logger().info(msg)
        
        # Start a thread for continuous publishing
        self.twist = Twist()
        self.publishing_thread = threading.Thread(target=self._publish_continuously)
        self.publishing_thread.daemon = True
        self.publishing_thread.start()
    
    def _publish_continuously(self):
        """Publish the twist message continuously at 10Hz"""
        rate = 0.1  # 10Hz
        while True:
            self.publisher.publish(self.twist)
            time.sleep(rate)
    
    def process_key(self, key):
        if key == 'w':
            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = 0.0
            self.get_logger().info('Forward')
        elif key == 'x':
            self.twist.linear.x = -self.linear_speed
            self.twist.angular.z = 0.0
            self.get_logger().info('Backward')
        elif key == 'a':
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.angular_speed
            self.get_logger().info('Left')
        elif key == 'd':
            self.twist.linear.x = 0.0
            self.twist.angular.z = -self.angular_speed
            self.get_logger().info('Right')
        elif key == 'q':
            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = self.angular_speed
            self.get_logger().info('Forward Left')
        elif key == 'e':
            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = -self.angular_speed
            self.get_logger().info('Forward Right')
        elif key == 'z':
            self.twist.linear.x = -self.linear_speed
            self.twist.angular.z = self.angular_speed
            self.get_logger().info('Backward Left')
        elif key == 'c':
            self.twist.linear.x = -self.linear_speed
            self.twist.angular.z = -self.angular_speed
            self.get_logger().info('Backward Right')
        elif key == 's':
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.get_logger().info('Stop')
        else:
            self.get_logger().info(f'Unknown key: {key}')

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    node = KeyboardControl()
    
    try:
        while True:
            key = get_key(settings)
            if key == '\x03':  # CTRL-C
                break
            node.process_key(key)
    except Exception as e:
        print(e)
    finally:
        # Stop the robot before exiting
        twist = Twist()
        node.publisher.publish(twist)
        
        # Clean up
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
