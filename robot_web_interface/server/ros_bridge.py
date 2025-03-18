#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from flask import Flask, jsonify, request
from flask_cors import CORS
import threading
import json
import numpy as np
import base64
import io
from PIL import Image, ImageDraw

# Initialize Flask app
app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Global variables to store ROS data
latest_map_data = None
latest_scan_data = None
latest_path_data = None
latest_robot_pose = None
available_topics = []
available_services = []

class ROSBridgeNode(Node):
    def __init__(self):
        super().__init__('ros_bridge_node')
        
        # Create subscribers
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
            
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        self.path_subscription = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10)
            
        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
            
        # Timer for updating topic and service lists
        self.timer = self.create_timer(5.0, self.update_ros_info)
        
        self.get_logger().info('ROS Bridge Node has been initialized')
        
    def map_callback(self, msg):
        global latest_map_data
        
        # Convert occupancy grid to image
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        
        # Create image from occupancy grid
        img = Image.new('RGB', (width, height), color='white')
        pixels = img.load()
        
        for i in range(width):
            for j in range(height):
                index = j * width + i
                if index < len(msg.data):
                    value = msg.data[index]
                    if value == -1:  # Unknown
                        pixels[i, j] = (128, 128, 128)
                    elif value == 0:  # Free
                        pixels[i, j] = (255, 255, 255)
                    else:  # Occupied
                        pixels[i, j] = (0, 0, 0)
        
        # Convert image to base64 string
        buffered = io.BytesIO()
        img.save(buffered, format="PNG")
        img_str = base64.b64encode(buffered.getvalue()).decode()
        
        latest_map_data = {
            'width': width,
            'height': height,
            'resolution': resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'image': img_str
        }
        
    def scan_callback(self, msg):
        global latest_scan_data
        
        # Convert scan data to list of points
        ranges = list(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        points = []
        for i, r in enumerate(ranges):
            if r >= msg.range_min and r <= msg.range_max:
                angle = angle_min + i * angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y])
        
        latest_scan_data = {
            'points': points,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        }
        
    def path_callback(self, msg):
        global latest_path_data
        
        # Convert path to list of points
        points = []
        for pose in msg.poses:
            points.append([
                pose.pose.position.x,
                pose.pose.position.y
            ])
        
        latest_path_data = {
            'points': points,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        }
        
    def update_ros_info(self):
        global available_topics, available_services
        
        # In a real implementation, you would use ROS2 introspection to get this info
        # For now, we'll just use a placeholder
        available_topics = [
            '/map',
            '/scan',
            '/path',
            '/cmd_vel',
            '/robot_pose'
        ]
        
        available_services = [
            '/navigate_to_goal',
            '/cancel_navigation'
        ]
        
        self.get_logger().info('Updated ROS info')

# Flask routes
@app.route('/api/topics', methods=['GET'])
def get_topics():
    return jsonify({'topics': available_topics})

@app.route('/api/services', methods=['GET'])
def get_services():
    return jsonify({'services': available_services})

@app.route('/api/map', methods=['GET'])
def get_map():
    if latest_map_data:
        return jsonify(latest_map_data)
    else:
        return jsonify({'error': 'No map data available'}), 404

@app.route('/api/scan', methods=['GET'])
def get_scan():
    if latest_scan_data:
        return jsonify(latest_scan_data)
    else:
        return jsonify({'error': 'No scan data available'}), 404

@app.route('/api/path', methods=['GET'])
def get_path():
    if latest_path_data:
        return jsonify(latest_path_data)
    else:
        return jsonify({'error': 'No path data available'}), 404

@app.route('/api/cmd_vel', methods=['POST'])
def publish_cmd_vel():
    data = request.json
    
    if not data or 'linear' not in data or 'angular' not in data:
        return jsonify({'error': 'Invalid request format'}), 400
    
    # Create and publish Twist message
    twist = Twist()
    twist.linear.x = float(data['linear'])
    twist.angular.z = float(data['angular'])
    
    ros_node.cmd_vel_publisher.publish(twist)
    
    return jsonify({'success': True})

@app.route('/api/navigate', methods=['POST'])
def navigate_to_goal():
    data = request.json
    
    if not data or 'goal' not in data:
        return jsonify({'error': 'Invalid request format'}), 400
    
    # In a real implementation, you would call a ROS2 service here
    # For now, we'll just return a success message
    
    return jsonify({'success': True, 'message': f"Navigation to {data['goal']} started"})

@app.route('/api/cancel_navigation', methods=['POST'])
def cancel_navigation():
    # In a real implementation, you would call a ROS2 service here
    # For now, we'll just return a success message
    
    return jsonify({'success': True, 'message': "Navigation cancelled"})

def run_flask():
    app.run(host='0.0.0.0', port=5000)

def main():
    global ros_node
    
    # Initialize ROS2
    rclpy.init()
    ros_node = ROSBridgeNode()
    
    # Start Flask in a separate thread
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    
    # Spin ROS2 node
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
