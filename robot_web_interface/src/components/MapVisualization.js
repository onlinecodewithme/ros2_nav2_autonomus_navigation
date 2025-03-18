import React, { useEffect, useRef, useState } from 'react';
import { 
  Box, 
  Typography, 
  Card, 
  CardContent, 
  CardHeader, 
  Grid,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Switch,
  FormControlLabel,
  Slider,
  Button
} from '@mui/material';
import * as THREE from 'three';
import ROSLIB from 'roslib';
import rosConnection from '../utils/rosConnection';

const MapVisualization = () => {
  const canvasRef = useRef(null);
  const rendererRef = useRef(null);
  const sceneRef = useRef(null);
  const cameraRef = useRef(null);
  const robotModelRef = useRef(null);
  const mapRef = useRef(null);
  const pathRef = useRef(null);
  const obstaclesRef = useRef(null);
  
  const [mapTopic, setMapTopic] = useState('/map');
  const [robotPoseTopic, setRobotPoseTopic] = useState('/robot_pose');
  const [scanTopic, setScanTopic] = useState('/scan');
  const [pathTopic, setPathTopic] = useState('/path');
  
  const [showMap, setShowMap] = useState(true);
  const [showRobot, setShowRobot] = useState(true);
  const [showScan, setShowScan] = useState(true);
  const [showPath, setShowPath] = useState(true);
  
  const [zoomLevel, setZoomLevel] = useState(50);
  const [availableTopics, setAvailableTopics] = useState([]);
  
  // Initialize Three.js scene
  useEffect(() => {
    if (!canvasRef.current) return;
    
    // Create scene
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf0f0f0);
    sceneRef.current = scene;
    
    // Create camera
    const camera = new THREE.PerspectiveCamera(
      75,
      canvasRef.current.clientWidth / canvasRef.current.clientHeight,
      0.1,
      1000
    );
    camera.position.z = 5;
    cameraRef.current = camera;
    
    // Create renderer
    const renderer = new THREE.WebGLRenderer({ canvas: canvasRef.current, antialias: true });
    renderer.setSize(canvasRef.current.clientWidth, canvasRef.current.clientHeight);
    rendererRef.current = renderer;
    
    // Add ambient light
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
    scene.add(ambientLight);
    
    // Add directional light
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
    directionalLight.position.set(0, 1, 0);
    scene.add(directionalLight);
    
    // Create robot model (simple cube for now)
    const robotGeometry = new THREE.BoxGeometry(0.5, 0.5, 0.2);
    const robotMaterial = new THREE.MeshStandardMaterial({ color: 0x2196f3 });
    const robot = new THREE.Mesh(robotGeometry, robotMaterial);
    scene.add(robot);
    robotModelRef.current = robot;
    
    // Create map placeholder
    const mapGeometry = new THREE.PlaneGeometry(10, 10);
    const mapMaterial = new THREE.MeshBasicMaterial({ 
      color: 0xffffff,
      transparent: true,
      opacity: 0.8
    });
    const map = new THREE.Mesh(mapGeometry, mapMaterial);
    map.rotation.x = -Math.PI / 2; // Lay flat on XZ plane
    map.position.y = -0.1; // Slightly below robot
    scene.add(map);
    mapRef.current = map;
    
    // Create path placeholder
    const pathMaterial = new THREE.LineBasicMaterial({ color: 0x4caf50, linewidth: 2 });
    const pathGeometry = new THREE.BufferGeometry();
    const pathLine = new THREE.Line(pathGeometry, pathMaterial);
    scene.add(pathLine);
    pathRef.current = pathLine;
    
    // Create obstacles group
    const obstacles = new THREE.Group();
    scene.add(obstacles);
    obstaclesRef.current = obstacles;
    
    // Animation loop
    const animate = () => {
      requestAnimationFrame(animate);
      renderer.render(scene, camera);
    };
    animate();
    
    // Handle window resize
    const handleResize = () => {
      if (!canvasRef.current) return;
      
      const width = canvasRef.current.clientWidth;
      const height = canvasRef.current.clientHeight;
      
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderer.setSize(width, height);
    };
    
    window.addEventListener('resize', handleResize);
    
    // Clean up
    return () => {
      window.removeEventListener('resize', handleResize);
      
      // Dispose of Three.js resources
      if (rendererRef.current) {
        rendererRef.current.dispose();
      }
      
      if (robotModelRef.current) {
        robotModelRef.current.geometry.dispose();
        robotModelRef.current.material.dispose();
      }
      
      if (mapRef.current) {
        mapRef.current.geometry.dispose();
        mapRef.current.material.dispose();
      }
      
      if (pathRef.current) {
        pathRef.current.geometry.dispose();
        pathRef.current.material.dispose();
      }
      
      if (obstaclesRef.current) {
        obstaclesRef.current.children.forEach(child => {
          child.geometry.dispose();
          child.material.dispose();
        });
      }
    };
  }, []);
  
  // Handle zoom level changes
  useEffect(() => {
    if (!cameraRef.current) return;
    
    // Convert zoom level (0-100) to camera position (10-1)
    const zoomFactor = 10 - (zoomLevel / 100) * 9;
    cameraRef.current.position.z = zoomFactor;
  }, [zoomLevel]);
  
  // Connect to ROS and subscribe to topics
  useEffect(() => {
    const handleConnection = () => {
      // Get available topics
      rosConnection.getTopics((topics) => {
        setAvailableTopics(topics.topics || []);
      });
      
      // Subscribe to map topic
      if (showMap) {
        subscribeToMap();
      }
      
      // Subscribe to robot pose topic
      if (showRobot) {
        subscribeToRobotPose();
      }
      
      // Subscribe to scan topic
      if (showScan) {
        subscribeToScan();
      }
      
      // Subscribe to path topic
      if (showPath) {
        subscribeToPath();
      }
    };
    
    rosConnection.events.on('connected', handleConnection);
    
    return () => {
      rosConnection.events.off('connected', handleConnection);
      
      // Unsubscribe from topics
      rosConnection.unsubscribe(mapTopic);
      rosConnection.unsubscribe(robotPoseTopic);
      rosConnection.unsubscribe(scanTopic);
      rosConnection.unsubscribe(pathTopic);
    };
  }, [mapTopic, robotPoseTopic, scanTopic, pathTopic]);
  
  // Subscribe to map topic
  const subscribeToMap = () => {
    if (!rosConnection.isConnected || !showMap) return;
    
    rosConnection.subscribe(
      mapTopic,
      'nav_msgs/OccupancyGrid',
      (message) => {
        if (!mapRef.current) return;
        
        // Create map texture from occupancy grid
        const width = message.info.width;
        const height = message.info.height;
        const resolution = message.info.resolution;
        
        // Create canvas for map texture
        const canvas = document.createElement('canvas');
        canvas.width = width;
        canvas.height = height;
        const context = canvas.getContext('2d');
        
        // Draw map data to canvas
        const imageData = context.createImageData(width, height);
        for (let i = 0; i < message.data.length; i++) {
          const value = message.data[i];
          const x = i % width;
          const y = Math.floor(i / width);
          const pixelIndex = (y * width + x) * 4;
          
          if (value === -1) {
            // Unknown (gray)
            imageData.data[pixelIndex] = 128;
            imageData.data[pixelIndex + 1] = 128;
            imageData.data[pixelIndex + 2] = 128;
            imageData.data[pixelIndex + 3] = 255;
          } else if (value === 0) {
            // Free space (white)
            imageData.data[pixelIndex] = 255;
            imageData.data[pixelIndex + 1] = 255;
            imageData.data[pixelIndex + 2] = 255;
            imageData.data[pixelIndex + 3] = 255;
          } else {
            // Occupied (black)
            imageData.data[pixelIndex] = 0;
            imageData.data[pixelIndex + 1] = 0;
            imageData.data[pixelIndex + 2] = 0;
            imageData.data[pixelIndex + 3] = 255;
          }
        }
        context.putImageData(imageData, 0, 0);
        
        // Create texture from canvas
        const texture = new THREE.CanvasTexture(canvas);
        texture.minFilter = THREE.LinearFilter;
        
        // Update map material
        mapRef.current.material.map = texture;
        mapRef.current.material.needsUpdate = true;
        
        // Update map size based on resolution
        const mapWidth = width * resolution;
        const mapHeight = height * resolution;
        mapRef.current.scale.set(mapWidth, mapHeight, 1);
        
        // Update map position
        const originX = message.info.origin.position.x;
        const originY = message.info.origin.position.y;
        mapRef.current.position.x = originX + mapWidth / 2;
        mapRef.current.position.z = originY + mapHeight / 2;
      }
    );
  };
  
  // Subscribe to robot pose topic
  const subscribeToRobotPose = () => {
    if (!rosConnection.isConnected || !showRobot) return;
    
    rosConnection.subscribe(
      robotPoseTopic,
      'geometry_msgs/PoseStamped',
      (message) => {
        if (!robotModelRef.current) return;
        
        const pose = message.pose;
        
        // Update robot position
        robotModelRef.current.position.x = pose.position.x;
        robotModelRef.current.position.z = pose.position.y;
        
        // Update robot orientation (convert quaternion to Euler angles)
        const quaternion = new THREE.Quaternion(
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w
        );
        robotModelRef.current.quaternion.copy(quaternion);
        
        // Rotate to align with ROS coordinate system
        robotModelRef.current.rotateX(Math.PI / 2);
      }
    );
  };
  
  // Subscribe to scan topic
  const subscribeToScan = () => {
    if (!rosConnection.isConnected || !showScan) return;
    
    rosConnection.subscribe(
      scanTopic,
      'sensor_msgs/LaserScan',
      (message) => {
        if (!obstaclesRef.current || !robotModelRef.current) return;
        
        // Clear previous scan points
        while (obstaclesRef.current.children.length > 0) {
          const child = obstaclesRef.current.children[0];
          child.geometry.dispose();
          child.material.dispose();
          obstaclesRef.current.remove(child);
        }
        
        // Create new scan points
        const angleMin = message.angle_min;
        const angleIncrement = message.angle_increment;
        const robotPosition = robotModelRef.current.position.clone();
        
        for (let i = 0; i < message.ranges.length; i++) {
          const range = message.ranges[i];
          
          // Skip invalid measurements
          if (range < message.range_min || range > message.range_max) {
            continue;
          }
          
          // Calculate point position
          const angle = angleMin + i * angleIncrement;
          const x = robotPosition.x + range * Math.cos(angle);
          const z = robotPosition.z + range * Math.sin(angle);
          
          // Create point geometry
          const pointGeometry = new THREE.SphereGeometry(0.05, 8, 8);
          const pointMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
          const point = new THREE.Mesh(pointGeometry, pointMaterial);
          
          point.position.set(x, robotPosition.y, z);
          obstaclesRef.current.add(point);
        }
      }
    );
  };
  
  // Subscribe to path topic
  const subscribeToPath = () => {
    if (!rosConnection.isConnected || !showPath) return;
    
    rosConnection.subscribe(
      pathTopic,
      'nav_msgs/Path',
      (message) => {
        if (!pathRef.current) return;
        
        const poses = message.poses;
        if (poses.length === 0) return;
        
        // Create path geometry
        const points = [];
        for (let i = 0; i < poses.length; i++) {
          const pose = poses[i].pose;
          points.push(new THREE.Vector3(pose.position.x, 0, pose.position.y));
        }
        
        // Update path geometry
        pathRef.current.geometry.dispose();
        pathRef.current.geometry = new THREE.BufferGeometry().setFromPoints(points);
      }
    );
  };
  
  // Handle topic changes
  useEffect(() => {
    if (rosConnection.isConnected) {
      // Unsubscribe from old topics
      rosConnection.unsubscribe(mapTopic);
      rosConnection.unsubscribe(robotPoseTopic);
      rosConnection.unsubscribe(scanTopic);
      rosConnection.unsubscribe(pathTopic);
      
      // Subscribe to new topics
      if (showMap) subscribeToMap();
      if (showRobot) subscribeToRobotPose();
      if (showScan) subscribeToScan();
      if (showPath) subscribeToPath();
    }
  }, [mapTopic, robotPoseTopic, scanTopic, pathTopic, showMap, showRobot, showScan, showPath]);
  
  // Handle visibility changes
  useEffect(() => {
    if (mapRef.current) {
      mapRef.current.visible = showMap;
    }
    
    if (robotModelRef.current) {
      robotModelRef.current.visible = showRobot;
    }
    
    if (obstaclesRef.current) {
      obstaclesRef.current.visible = showScan;
    }
    
    if (pathRef.current) {
      pathRef.current.visible = showPath;
    }
    
    // Update subscriptions
    if (rosConnection.isConnected) {
      if (showMap) {
        subscribeToMap();
      } else {
        rosConnection.unsubscribe(mapTopic);
      }
      
      if (showRobot) {
        subscribeToRobotPose();
      } else {
        rosConnection.unsubscribe(robotPoseTopic);
      }
      
      if (showScan) {
        subscribeToScan();
      } else {
        rosConnection.unsubscribe(scanTopic);
      }
      
      if (showPath) {
        subscribeToPath();
      } else {
        rosConnection.unsubscribe(pathTopic);
      }
    }
  }, [showMap, showRobot, showScan, showPath]);
  
  // Reset camera view
  const handleResetView = () => {
    if (!cameraRef.current) return;
    
    cameraRef.current.position.set(0, 5, 5);
    cameraRef.current.lookAt(0, 0, 0);
    setZoomLevel(50);
  };
  
  // Filter topics by type
  const filterTopicsByType = (type) => {
    return availableTopics.filter(topic => {
      // This is a simplistic approach - in a real app, you'd use ROS topic types
      if (type === 'map' && topic.includes('map')) return true;
      if (type === 'pose' && (topic.includes('pose') || topic.includes('position'))) return true;
      if (type === 'scan' && topic.includes('scan')) return true;
      if (type === 'path' && topic.includes('path')) return true;
      return false;
    });
  };
  
  return (
    <Box sx={{ flexGrow: 1, mt: 2 }}>
      <Typography variant="h4" gutterBottom>
        Map Visualization
      </Typography>
      
      <Grid container spacing={3}>
        {/* Visualization Canvas */}
        <Grid item xs={12} md={8}>
          <Card sx={{ height: '100%' }}>
            <CardContent sx={{ height: '500px', position: 'relative' }}>
              <canvas 
                ref={canvasRef} 
                style={{ width: '100%', height: '100%' }}
              />
              
              <Box sx={{ position: 'absolute', bottom: 16, right: 16 }}>
                <Button 
                  variant="contained" 
                  color="primary"
                  onClick={handleResetView}
                >
                  Reset View
                </Button>
              </Box>
            </CardContent>
          </Card>
        </Grid>
        
        {/* Controls */}
        <Grid item xs={12} md={4}>
          <Card>
            <CardHeader title="Visualization Controls" />
            <CardContent>
              <Typography gutterBottom>Zoom Level</Typography>
              <Slider
                value={zoomLevel}
                min={0}
                max={100}
                step={1}
                onChange={(_, value) => setZoomLevel(value)}
                valueLabelDisplay="auto"
                aria-label="Zoom Level"
              />
              
              <Box sx={{ mt: 3 }}>
                <Typography gutterBottom>Layer Visibility</Typography>
                <FormControlLabel
                  control={
                    <Switch
                      checked={showMap}
                      onChange={(e) => setShowMap(e.target.checked)}
                    />
                  }
                  label="Show Map"
                />
                <FormControlLabel
                  control={
                    <Switch
                      checked={showRobot}
                      onChange={(e) => setShowRobot(e.target.checked)}
                    />
                  }
                  label="Show Robot"
                />
                <FormControlLabel
                  control={
                    <Switch
                      checked={showScan}
                      onChange={(e) => setShowScan(e.target.checked)}
                    />
                  }
                  label="Show Laser Scan"
                />
                <FormControlLabel
                  control={
                    <Switch
                      checked={showPath}
                      onChange={(e) => setShowPath(e.target.checked)}
                    />
                  }
                  label="Show Path"
                />
              </Box>
              
              <Box sx={{ mt: 3 }}>
                <Typography gutterBottom>Topic Selection</Typography>
                <FormControl fullWidth sx={{ mb: 2 }}>
                  <InputLabel id="map-topic-label">Map Topic</InputLabel>
                  <Select
                    labelId="map-topic-label"
                    value={mapTopic}
                    label="Map Topic"
                    onChange={(e) => setMapTopic(e.target.value)}
                  >
                    {filterTopicsByType('map').map((topic) => (
                      <MenuItem key={topic} value={topic}>
                        {topic}
                      </MenuItem>
                    ))}
                  </Select>
                </FormControl>
                
                <FormControl fullWidth sx={{ mb: 2 }}>
                  <InputLabel id="robot-pose-topic-label">Robot Pose Topic</InputLabel>
                  <Select
                    labelId="robot-pose-topic-label"
                    value={robotPoseTopic}
                    label="Robot Pose Topic"
                    onChange={(e) => setRobotPoseTopic(e.target.value)}
                  >
                    {filterTopicsByType('pose').map((topic) => (
                      <MenuItem key={topic} value={topic}>
                        {topic}
                      </MenuItem>
                    ))}
                  </Select>
                </FormControl>
                
                <FormControl fullWidth sx={{ mb: 2 }}>
                  <InputLabel id="scan-topic-label">Scan Topic</InputLabel>
                  <Select
                    labelId="scan-topic-label"
                    value={scanTopic}
                    label="Scan Topic"
                    onChange={(e) => setScanTopic(e.target.value)}
                  >
                    {filterTopicsByType('scan').map((topic) => (
                      <MenuItem key={topic} value={topic}>
                        {topic}
                      </MenuItem>
                    ))}
                  </Select>
                </FormControl>
                
                <FormControl fullWidth>
                  <InputLabel id="path-topic-label">Path Topic</InputLabel>
                  <Select
                    labelId="path-topic-label"
                    value={pathTopic}
                    label="Path Topic"
                    onChange={(e) => setPathTopic(e.target.value)}
                  >
                    {filterTopicsByType('path').map((topic) => (
                      <MenuItem key={topic} value={topic}>
                        {topic}
                      </MenuItem>
                    ))}
                  </Select>
                </FormControl>
              </Box>
            </CardContent>
          </Card>
        </Grid>
      </Grid>
    </Box>
  );
};

export default MapVisualization;
