#!/bin/bash

# Make the script executable
chmod +x server/ros_bridge.py

# Start the Python server in the background
echo "Starting ROS Bridge server..."
cd server
python3 -m pip install -r requirements.txt
python3 ros_bridge.py &
SERVER_PID=$!
cd ..

# Wait for the server to start
sleep 2
echo "ROS Bridge server started on http://localhost:5000"

# Start the React application
echo "Starting React application..."
npm start -- --port 3002

# When the React app is terminated, also terminate the Python server
kill $SERVER_PID
