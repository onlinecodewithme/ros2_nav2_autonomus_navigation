#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
import time

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf_py')
        
        # Parameters
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('parent_frame', 'odom')  # Changed from 'map' to 'odom'
        self.declare_parameter('child_frame', 'base_link')
        
        self.odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        
        # Create TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create a static transform broadcaster for the map frame
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Publish a static transform from map to odom
        self.publish_map_to_odom()
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10)
        
        self.get_logger().info('Odometry to TF node initialized')
        self.get_logger().info(f'Subscribing to: {self.odom_topic}')
        self.get_logger().info(f'Publishing transform from {self.parent_frame} to {self.child_frame}')
    
    def publish_map_to_odom(self):
        """Publish a static transform from map to odom"""
        # First publish a static transform from world to map
        world_to_map = TransformStamped()
        world_to_map.header.stamp = self.get_clock().now().to_msg()
        world_to_map.header.frame_id = "world"  # Parent frame is world
        world_to_map.child_frame_id = "map"  # Child frame is map
        
        # Identity transform
        world_to_map.transform.translation.x = 0.0
        world_to_map.transform.translation.y = 0.0
        world_to_map.transform.translation.z = 0.0
        world_to_map.transform.rotation.x = 0.0
        world_to_map.transform.rotation.y = 0.0
        world_to_map.transform.rotation.z = 0.0
        world_to_map.transform.rotation.w = 1.0
        
        # Publish static transform
        self.static_tf_broadcaster.sendTransform(world_to_map)
        self.get_logger().info('Published static transform: world -> map')
        
        # Now publish a static transform from map to odom
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = self.get_clock().now().to_msg()
        map_to_odom.header.frame_id = "map"  # Parent frame is map
        map_to_odom.child_frame_id = "odom"  # Child frame is odom
        
        # Identity transform
        map_to_odom.transform.translation.x = 0.0
        map_to_odom.transform.translation.y = 0.0
        map_to_odom.transform.translation.z = 0.0
        map_to_odom.transform.rotation.x = 0.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0
        
        # Publish static transform
        self.static_tf_broadcaster.sendTransform(map_to_odom)
        self.get_logger().info('Published static transform: map -> odom')
        
        # Also publish a static transform from odom to base_link
        self.publish_odom_to_base_link()
    
    def publish_odom_to_base_link(self):
        """Publish a static transform from odom to base_link"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"  # Parent frame is odom
        transform.child_frame_id = "base_link"  # Child frame is base_link
        
        # Identity transform
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        # Publish static transform
        self.static_tf_broadcaster.sendTransform(transform)
        self.get_logger().info('Published static transform: odom -> base_link')
    
    def odom_callback(self, msg):
        transform = TransformStamped()
        
        # Set header
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.parent_frame
        
        # Set child frame
        transform.child_frame_id = self.child_frame
        
        # Set translation
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        
        # Set rotation
        transform.transform.rotation = msg.pose.pose.orientation
        
        # Publish transform
        self.tf_broadcaster.sendTransform(transform)
        
        # Republish the transforms periodically
        if hasattr(self, 'last_map_publish_time'):
            current_time = time.time()
            if current_time - self.last_map_publish_time > 1.0:  # Republish every second
                self.publish_map_to_odom()
                self.last_map_publish_time = current_time
        else:
            self.last_map_publish_time = time.time()
        
        # Log for debugging
        self.get_logger().debug(f'Published transform: {self.parent_frame} -> {self.child_frame}')
        self.get_logger().debug(f'Position: ({transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z})')

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
