import React, { useState, useEffect, useRef } from 'react';
import { 
  Box, 
  Typography, 
  Grid, 
  Card, 
  CardContent, 
  CardHeader, 
  Slider, 
  Button,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Switch,
  FormControlLabel,
  Alert
} from '@mui/material';
import rosConnection from '../utils/rosConnection';

const RobotControl = () => {
  const [linearVelocity, setLinearVelocity] = useState(0);
  const [angularVelocity, setAngularVelocity] = useState(0);
  const [maxLinearVelocity, setMaxLinearVelocity] = useState(1.0);
  const [maxAngularVelocity, setMaxAngularVelocity] = useState(1.0);
  const [controlMode, setControlMode] = useState('manual');
  const [isEmergencyStop, setIsEmergencyStop] = useState(false);
  const [navigationGoal, setNavigationGoal] = useState('');
  const [availableGoals, setAvailableGoals] = useState([]);
  const [statusMessage, setStatusMessage] = useState('');
  const [showStatusAlert, setShowStatusAlert] = useState(false);
  const [statusSeverity, setStatusSeverity] = useState('info');
  
  const joystickRef = useRef(null);
  const joystickKnobRef = useRef(null);
  const joystickActive = useRef(false);
  const joystickCenter = useRef({ x: 0, y: 0 });
  const joystickPosition = useRef({ x: 0, y: 0 });
  
  // Initialize joystick control
  useEffect(() => {
    if (joystickRef.current && joystickKnobRef.current) {
      const joystick = joystickRef.current;
      const knob = joystickKnobRef.current;
      
      const rect = joystick.getBoundingClientRect();
      joystickCenter.current = {
        x: rect.width / 2,
        y: rect.height / 2
      };
      
      const handleMouseDown = (e) => {
        joystickActive.current = true;
        handleMouseMove(e);
      };
      
      const handleMouseMove = (e) => {
        if (!joystickActive.current) return;
        
        const rect = joystick.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        // Calculate distance from center
        const deltaX = x - joystickCenter.current.x;
        const deltaY = y - joystickCenter.current.y;
        const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        // Limit to joystick bounds (radius)
        const maxDistance = rect.width / 2 - knob.offsetWidth / 2;
        const limitedDistance = Math.min(distance, maxDistance);
        
        // Calculate angle
        const angle = Math.atan2(deltaY, deltaX);
        
        // Calculate new position
        const newX = joystickCenter.current.x + limitedDistance * Math.cos(angle);
        const newY = joystickCenter.current.y + limitedDistance * Math.sin(angle);
        
        // Update knob position
        knob.style.left = `${newX}px`;
        knob.style.top = `${newY}px`;
        
        // Calculate velocities
        const normalizedX = deltaX / maxDistance;
        const normalizedY = -deltaY / maxDistance; // Invert Y axis
        
        joystickPosition.current = {
          x: normalizedX,
          y: normalizedY
        };
        
        // Update velocity values
        setLinearVelocity(normalizedY * maxLinearVelocity);
        setAngularVelocity(-normalizedX * maxAngularVelocity);
      };
      
      const handleMouseUp = () => {
        if (!joystickActive.current) return;
        
        joystickActive.current = false;
        
        // Reset knob position
        knob.style.left = `${joystickCenter.current.x}px`;
        knob.style.top = `${joystickCenter.current.y}px`;
        
        joystickPosition.current = { x: 0, y: 0 };
        
        // Reset velocity values
        setLinearVelocity(0);
        setAngularVelocity(0);
      };
      
      // Add event listeners
      joystick.addEventListener('mousedown', handleMouseDown);
      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);
      
      // Clean up
      return () => {
        joystick.removeEventListener('mousedown', handleMouseDown);
        document.removeEventListener('mousemove', handleMouseMove);
        document.removeEventListener('mouseup', handleMouseUp);
      };
    }
  }, [maxLinearVelocity, maxAngularVelocity]);
  
  // Publish velocity commands
  useEffect(() => {
    if (isEmergencyStop) {
      // Send zero velocity command
      publishVelocityCommand(0, 0);
      return;
    }
    
    if (controlMode === 'manual') {
      // Only publish when joystick is active
      if (joystickActive.current) {
        publishVelocityCommand(linearVelocity, angularVelocity);
      }
    }
  }, [linearVelocity, angularVelocity, isEmergencyStop, controlMode]);
  
  // Connect to ROS and set up subscriptions
  useEffect(() => {
    const handleConnection = () => {
      // Subscribe to available navigation goals
      rosConnection.subscribe(
        '/navigation/available_goals',
        'std_msgs/String[]',
        (message) => {
          setAvailableGoals(message.data || []);
        }
      );
      
      // Subscribe to navigation status
      rosConnection.subscribe(
        '/navigation/status',
        'std_msgs/String',
        (message) => {
          setStatusMessage(message.data);
          setShowStatusAlert(true);
          
          // Hide alert after 5 seconds
          setTimeout(() => {
            setShowStatusAlert(false);
          }, 5000);
        }
      );
    };
    
    rosConnection.events.on('connected', handleConnection);
    
    return () => {
      rosConnection.events.off('connected', handleConnection);
      rosConnection.unsubscribe('/navigation/available_goals');
      rosConnection.unsubscribe('/navigation/status');
    };
  }, []);
  
  // Function to publish velocity commands
  const publishVelocityCommand = (linear, angular) => {
    if (!rosConnection.isConnected) return;
    
    rosConnection.publish(
      '/cmd_vel',
      'geometry_msgs/Twist',
      {
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular }
      }
    );
  };
  
  // Handle emergency stop
  const handleEmergencyStop = () => {
    setIsEmergencyStop(true);
    publishVelocityCommand(0, 0);
    
    setStatusMessage('Emergency stop activated');
    setStatusSeverity('error');
    setShowStatusAlert(true);
  };
  
  // Handle emergency stop release
  const handleReleaseEmergencyStop = () => {
    setIsEmergencyStop(false);
    
    setStatusMessage('Emergency stop released');
    setStatusSeverity('success');
    setShowStatusAlert(true);
  };
  
  // Handle navigation goal selection
  const handleNavigationGoalChange = (event) => {
    setNavigationGoal(event.target.value);
  };
  
  // Handle navigation start
  const handleStartNavigation = () => {
    if (!navigationGoal) return;
    
    setControlMode('autonomous');
    
    // Call navigation service
    rosConnection.callService(
      '/navigation/navigate_to_goal',
      'std_srvs/SetString',
      { data: navigationGoal },
      (result) => {
        if (result && result.success) {
          setStatusMessage(`Navigation to ${navigationGoal} started`);
          setStatusSeverity('info');
        } else {
          setStatusMessage(`Failed to start navigation: ${result ? result.message : 'Unknown error'}`);
          setStatusSeverity('error');
        }
        setShowStatusAlert(true);
      }
    );
  };
  
  // Handle navigation stop
  const handleStopNavigation = () => {
    // Call cancel navigation service
    rosConnection.callService(
      '/navigation/cancel_navigation',
      'std_srvs/Trigger',
      {},
      (result) => {
        if (result && result.success) {
          setStatusMessage('Navigation cancelled');
          setStatusSeverity('warning');
          setControlMode('manual');
        } else {
          setStatusMessage(`Failed to cancel navigation: ${result ? result.message : 'Unknown error'}`);
          setStatusSeverity('error');
        }
        setShowStatusAlert(true);
      }
    );
  };
  
  return (
    <Box sx={{ flexGrow: 1, mt: 2 }}>
      <Typography variant="h4" gutterBottom>
        Robot Control
      </Typography>
      
      {showStatusAlert && (
        <Alert 
          severity={statusSeverity} 
          sx={{ mb: 2 }}
          onClose={() => setShowStatusAlert(false)}
        >
          {statusMessage}
        </Alert>
      )}
      
      <Grid container spacing={3}>
        {/* Control Mode */}
        <Grid item xs={12}>
          <Card>
            <CardHeader title="Control Mode" />
            <CardContent>
              <Grid container spacing={2} alignItems="center">
                <Grid item xs={12} sm={6}>
                  <FormControl fullWidth>
                    <InputLabel id="control-mode-label">Control Mode</InputLabel>
                    <Select
                      labelId="control-mode-label"
                      value={controlMode}
                      label="Control Mode"
                      onChange={(e) => setControlMode(e.target.value)}
                      disabled={isEmergencyStop}
                    >
                      <MenuItem value="manual">Manual Control</MenuItem>
                      <MenuItem value="autonomous">Autonomous Navigation</MenuItem>
                    </Select>
                  </FormControl>
                </Grid>
                <Grid item xs={12} sm={6}>
                  {isEmergencyStop ? (
                    <Button 
                      variant="contained" 
                      color="success" 
                      fullWidth
                      onClick={handleReleaseEmergencyStop}
                    >
                      Release Emergency Stop
                    </Button>
                  ) : (
                    <Button 
                      variant="contained" 
                      color="error" 
                      fullWidth
                      onClick={handleEmergencyStop}
                    >
                      Emergency Stop
                    </Button>
                  )}
                </Grid>
              </Grid>
            </CardContent>
          </Card>
        </Grid>
        
        {/* Manual Control */}
        <Grid item xs={12} md={6}>
          <Card>
            <CardHeader title="Manual Control" />
            <CardContent>
              <Box sx={{ mb: 3 }}>
                <Typography gutterBottom>Joystick Control</Typography>
                <Box 
                  className="joystick-container" 
                  ref={joystickRef}
                  sx={{ 
                    opacity: controlMode === 'manual' && !isEmergencyStop ? 1 : 0.5,
                    pointerEvents: controlMode === 'manual' && !isEmergencyStop ? 'auto' : 'none'
                  }}
                >
                  <Box 
                    className="joystick-knob" 
                    ref={joystickKnobRef}
                    sx={{ 
                      position: 'absolute',
                      top: '50%',
                      left: '50%',
                      transform: 'translate(-50%, -50%)'
                    }}
                  />
                </Box>
              </Box>
              
              <Box sx={{ mb: 2 }}>
                <Typography gutterBottom>Linear Velocity: {linearVelocity.toFixed(2)} m/s</Typography>
                <Slider
                  value={maxLinearVelocity}
                  min={0.1}
                  max={2.0}
                  step={0.1}
                  onChange={(_, value) => setMaxLinearVelocity(value)}
                  valueLabelDisplay="auto"
                  valueLabelFormat={(value) => `${value.toFixed(1)} m/s`}
                  disabled={controlMode !== 'manual' || isEmergencyStop}
                  aria-label="Max Linear Velocity"
                />
                <Typography variant="caption" color="text.secondary">
                  Max Linear Velocity
                </Typography>
              </Box>
              
              <Box>
                <Typography gutterBottom>Angular Velocity: {angularVelocity.toFixed(2)} rad/s</Typography>
                <Slider
                  value={maxAngularVelocity}
                  min={0.1}
                  max={2.0}
                  step={0.1}
                  onChange={(_, value) => setMaxAngularVelocity(value)}
                  valueLabelDisplay="auto"
                  valueLabelFormat={(value) => `${value.toFixed(1)} rad/s`}
                  disabled={controlMode !== 'manual' || isEmergencyStop}
                  aria-label="Max Angular Velocity"
                />
                <Typography variant="caption" color="text.secondary">
                  Max Angular Velocity
                </Typography>
              </Box>
            </CardContent>
          </Card>
        </Grid>
        
        {/* Autonomous Navigation */}
        <Grid item xs={12} md={6}>
          <Card>
            <CardHeader title="Autonomous Navigation" />
            <CardContent>
              <Box sx={{ mb: 3 }}>
                <FormControl fullWidth sx={{ mb: 2 }}>
                  <InputLabel id="navigation-goal-label">Navigation Goal</InputLabel>
                  <Select
                    labelId="navigation-goal-label"
                    value={navigationGoal}
                    label="Navigation Goal"
                    onChange={handleNavigationGoalChange}
                    disabled={controlMode !== 'autonomous' || isEmergencyStop}
                  >
                    {availableGoals.length > 0 ? (
                      availableGoals.map((goal) => (
                        <MenuItem key={goal} value={goal}>
                          {goal}
                        </MenuItem>
                      ))
                    ) : (
                      <MenuItem value="" disabled>
                        No goals available
                      </MenuItem>
                    )}
                  </Select>
                </FormControl>
                
                <Grid container spacing={2}>
                  <Grid item xs={6}>
                    <Button
                      variant="contained"
                      color="primary"
                      fullWidth
                      onClick={handleStartNavigation}
                      disabled={!navigationGoal || isEmergencyStop || controlMode !== 'autonomous'}
                    >
                      Start Navigation
                    </Button>
                  </Grid>
                  <Grid item xs={6}>
                    <Button
                      variant="outlined"
                      color="secondary"
                      fullWidth
                      onClick={handleStopNavigation}
                      disabled={controlMode !== 'autonomous' || isEmergencyStop}
                    >
                      Stop Navigation
                    </Button>
                  </Grid>
                </Grid>
              </Box>
              
              <Box>
                <FormControlLabel
                  control={
                    <Switch
                      checked={controlMode === 'autonomous'}
                      onChange={(e) => setControlMode(e.target.checked ? 'autonomous' : 'manual')}
                      disabled={isEmergencyStop}
                    />
                  }
                  label="Enable Autonomous Mode"
                />
              </Box>
            </CardContent>
          </Card>
        </Grid>
      </Grid>
    </Box>
  );
};

export default RobotControl;
