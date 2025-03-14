#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

msg = """
Control Your Robot!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

w/x : increase/decrease linear velocity (0.1 m/s)
a/d : increase/decrease angular velocity (0.1 rad/s)
s : stop
q/e/z/c : diagonal movement

CTRL-C to quit
"""

moveBindings = {
    'w': (0.1, 0.0),
    'x': (-0.1, 0.0),
    'a': (0.0, 0.1),
    'd': (0.0, -0.1),
    'q': (0.1, 0.1),
    'e': (0.1, -0.1),
    'z': (-0.1, 0.1),
    'c': (-0.1, -0.1),
    's': (0.0, 0.0),
}

class SimpleKeyTeleop(Node):
    def __init__(self):
        super().__init__('simple_key_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.get_logger().info('Simple teleop node started')
        self.get_logger().info(msg)
        
    def publish_cmd(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher.publish(twist)
        self.get_logger().info(f'Linear: {self.linear_speed:.2f}, Angular: {self.angular_speed:.2f}')

def getKey(settings):
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
    teleop = SimpleKeyTeleop()
    
    try:
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                linear_increment, angular_increment = moveBindings[key]
                teleop.linear_speed += linear_increment
                teleop.angular_speed += angular_increment
                teleop.publish_cmd()
            elif key == '\x03':  # CTRL-C
                break
    except Exception as e:
        print(e)
    finally:
        # Stop the robot
        teleop.linear_speed = 0.0
        teleop.angular_speed = 0.0
        teleop.publish_cmd()
        
        # Clean up
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
