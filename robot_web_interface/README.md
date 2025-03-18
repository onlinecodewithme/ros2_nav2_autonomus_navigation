# Robot Web Interface

A web-based interface for controlling and monitoring ROS-based robots. This application provides a user-friendly interface for visualizing robot data, controlling the robot, and monitoring its status.

## Features

- **Dashboard**: View system status, available ROS topics and services, and recent activity
- **Robot Control**: Manual control with joystick interface and autonomous navigation to predefined goals
- **Map Visualization**: View the robot's environment map, laser scan data, and planned path
- **Settings**: Configure ROS connection settings and interface preferences

## Architecture

The application consists of two main components:

1. **React Frontend**: A web application built with React.js, Material-UI, and Three.js for visualization
2. **Python Backend**: A ROS bridge server that connects to ROS and exposes REST API endpoints for the frontend

### Frontend

The frontend is built with:

- React.js for UI components
- Material-UI for styling
- Three.js for 3D visualization
- React Router for navigation
- Fetch API for communication with the backend

### Backend

The backend is built with:

- Flask for the REST API
- ROS2 Python client library for communication with ROS
- PIL for image processing

## Prerequisites

- Node.js (v12 or later)
- Python 3.8 or later
- ROS2 (Humble or later)
- ZED SDK (if using ZED camera)

## Installation

1. Clone the repository
2. Install frontend dependencies:
   ```
   npm install
   ```
3. Install backend dependencies:
   ```
   cd server
   pip install -r requirements.txt
   ```

## Usage

### Starting the Application

Use the provided start script to launch both the backend and frontend:

```bash
./start.sh
```

This will:
1. Start the Python ROS bridge server on port 5000
2. Start the React development server on port 3001
3. Open the application in your default web browser

### Manual Start

If you prefer to start the components separately:

1. Start the ROS bridge server:
   ```bash
   cd server
   python ros_bridge.py
   ```

2. Start the React application:
   ```bash
   npm start -- --port 3001
   ```

## Development

### Frontend Structure

- `src/components/`: React components
- `src/utils/`: Utility functions and services
- `src/styles.css`: Global styles

### Backend Structure

- `server/ros_bridge.py`: Main server file with ROS node and Flask app
- `server/requirements.txt`: Python dependencies

## Troubleshooting

### Connection Issues

If the application cannot connect to ROS:

1. Check if the ROS bridge server is running
2. Verify that ROS is properly set up and running
3. Check the console for error messages

### Visualization Issues

If the map or other visualizations are not displaying correctly:

1. Check if the corresponding ROS topics are being published
2. Verify that the topic names in the application match the actual ROS topics
3. Check the browser console for error messages

## License

This project is licensed under the MIT License - see the LICENSE file for details.
