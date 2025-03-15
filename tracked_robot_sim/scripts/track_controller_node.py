#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class TrackedRobotController(Node):
    """Controller for the tracked robot, supports keyboard, joystick, and autonomous operation."""
    
    def __init__(self):
        super().__init__('tracked_robot_controller')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('control_mode', 'keyboard')  # 'keyboard', 'joystick', or 'autonomous'
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.control_mode = self.get_parameter('control_mode').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/tracked_robot/cmd_vel', 10)
        self.left_track_pub = self.create_publisher(
            Float64, '/tracked_robot/left_track_velocity_controller/command', 10)
        self.right_track_pub = self.create_publisher(
            Float64, '/tracked_robot/right_track_velocity_controller/command', 10)
        
        # Subscribers based on control mode
        if self.control_mode == 'joystick':
            self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Current velocity commands
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        
        self.get_logger().info(f'Tracked robot controller initialized in {self.control_mode} mode')
        
        if self.control_mode == 'keyboard':
            self.get_logger().info('Use teleop_twist_keyboard to control the robot')
            self.get_logger().info('Run: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/tracked_robot/cmd_vel')
    
    def joy_callback(self, msg):
        """Process joystick input."""
        # Map joystick axes to velocity commands
        # Assuming left stick for forward/backward and right stick for rotation
        self.current_linear_x = msg.axes[1] * self.max_linear_speed
        self.current_angular_z = msg.axes[3] * self.max_angular_speed
    
    def cmd_vel_callback(self, msg):
        """Process velocity commands."""
        self.current_linear_x = msg.linear.x
        self.current_angular_z = msg.angular.z
    
    def control_loop(self):
        """Main control loop for the robot."""
        # Create Twist message
        cmd_vel = Twist()
        cmd_vel.linear.x = self.current_linear_x
        cmd_vel.angular.z = self.current_angular_z
        
        # Publish to cmd_vel topic
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Calculate track velocities from linear and angular commands
        # For a differential drive, the left and right wheel velocities are:
        # v_left = v_linear - 0.5 * v_angular * track_width
        # v_right = v_linear + 0.5 * v_angular * track_width
        track_width = 0.5  # Distance between tracks in meters
        
        left_track_velocity = Float64()
        right_track_velocity = Float64()
        
        left_track_velocity.data = self.current_linear_x - (0.5 * self.current_angular_z * track_width)
        right_track_velocity.data = self.current_linear_x + (0.5 * self.current_angular_z * track_width)
        
        # Publish track velocities
        self.left_track_pub.publish(left_track_velocity)
        self.right_track_pub.publish(right_track_velocity)

def main(args=None):
    rclpy.init(args=args)
    controller = TrackedRobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()