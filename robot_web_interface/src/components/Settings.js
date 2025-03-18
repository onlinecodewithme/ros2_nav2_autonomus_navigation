import React, { useState, useEffect } from 'react';
import { 
  Box, 
  Typography, 
  Card, 
  CardContent, 
  CardHeader, 
  Grid,
  TextField,
  Button,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Switch,
  FormControlLabel,
  Divider,
  Alert,
  Snackbar
} from '@mui/material';
import rosConnection from '../utils/rosConnection';

const Settings = () => {
  const [rosUrl, setRosUrl] = useState('ws://localhost:9090');
  const [autoConnect, setAutoConnect] = useState(true);
  const [connectionStatus, setConnectionStatus] = useState('disconnected');
  const [snackbarOpen, setSnackbarOpen] = useState(false);
  const [snackbarMessage, setSnackbarMessage] = useState('');
  const [snackbarSeverity, setSnackbarSeverity] = useState('info');
  const [availableTopics, setAvailableTopics] = useState([]);
  const [availableServices, setAvailableServices] = useState([]);
  const [topicFilter, setTopicFilter] = useState('');
  const [serviceFilter, setServiceFilter] = useState('');
  
  // Connect to ROS
  useEffect(() => {
    const handleStatusChange = (status) => {
      setConnectionStatus(status);
    };
    
    rosConnection.events.on('status_change', handleStatusChange);
    
    // Auto-connect if enabled
    if (autoConnect) {
      handleConnect();
    }
    
    return () => {
      rosConnection.events.off('status_change', handleStatusChange);
    };
  }, []);
  
  // Handle connection
  const handleConnect = () => {
    rosConnection.connect(rosUrl);
    
    // Show snackbar
    setSnackbarMessage('Connecting to ROS bridge server...');
    setSnackbarSeverity('info');
    setSnackbarOpen(true);
    
    // Set up event listeners for connection status
    const handleConnected = () => {
      setSnackbarMessage('Connected to ROS bridge server');
      setSnackbarSeverity('success');
      setSnackbarOpen(true);
      
      // Get available topics and services
      rosConnection.getTopics((topics) => {
        setAvailableTopics(topics.topics || []);
      });
      
      rosConnection.getServices((services) => {
        setAvailableServices(services || []);
      });
    };
    
    const handleDisconnected = () => {
      setSnackbarMessage('Disconnected from ROS bridge server');
      setSnackbarSeverity('warning');
      setSnackbarOpen(true);
    };
    
    const handleError = (error) => {
      setSnackbarMessage(`Error connecting to ROS bridge server: ${error.message}`);
      setSnackbarSeverity('error');
      setSnackbarOpen(true);
    };
    
    rosConnection.events.once('connected', handleConnected);
    rosConnection.events.once('disconnected', handleDisconnected);
    rosConnection.events.once('error', handleError);
  };
  
  // Handle disconnection
  const handleDisconnect = () => {
    rosConnection.disconnect();
  };
  
  // Handle URL change
  const handleUrlChange = (event) => {
    setRosUrl(event.target.value);
  };
  
  // Handle auto-connect change
  const handleAutoConnectChange = (event) => {
    setAutoConnect(event.target.checked);
  };
  
  // Handle snackbar close
  const handleSnackbarClose = (event, reason) => {
    if (reason === 'clickaway') {
      return;
    }
    setSnackbarOpen(false);
  };
  
  // Handle topic filter change
  const handleTopicFilterChange = (event) => {
    setTopicFilter(event.target.value);
  };
  
  // Handle service filter change
  const handleServiceFilterChange = (event) => {
    setServiceFilter(event.target.value);
  };
  
  // Filter topics
  const filteredTopics = availableTopics.filter((topic) => {
    return topic.toLowerCase().includes(topicFilter.toLowerCase());
  });
  
  // Filter services
  const filteredServices = availableServices.filter((service) => {
    return service.toLowerCase().includes(serviceFilter.toLowerCase());
  });
  
  return (
    <Box sx={{ flexGrow: 1, mt: 2 }}>
      <Typography variant="h4" gutterBottom>
        Settings
      </Typography>
      
      <Grid container spacing={3}>
        {/* Connection Settings */}
        <Grid item xs={12} md={6}>
          <Card>
            <CardHeader title="ROS Connection Settings" />
            <CardContent>
              <Box sx={{ mb: 3 }}>
                <TextField
                  fullWidth
                  label="ROS Bridge URL"
                  variant="outlined"
                  value={rosUrl}
                  onChange={handleUrlChange}
                  helperText="Example: ws://localhost:9090"
                  sx={{ mb: 2 }}
                />
                
                <FormControlLabel
                  control={
                    <Switch
                      checked={autoConnect}
                      onChange={handleAutoConnectChange}
                    />
                  }
                  label="Auto-connect on startup"
                />
              </Box>
              
              <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                <Button
                  variant="contained"
                  color="primary"
                  onClick={handleConnect}
                  disabled={connectionStatus === 'connected' || connectionStatus === 'connecting'}
                >
                  Connect
                </Button>
                
                <Button
                  variant="outlined"
                  color="secondary"
                  onClick={handleDisconnect}
                  disabled={connectionStatus === 'disconnected'}
                >
                  Disconnect
                </Button>
              </Box>
              
              <Box sx={{ mt: 2 }}>
                <Alert severity={
                  connectionStatus === 'connected' ? 'success' :
                  connectionStatus === 'connecting' ? 'info' : 'error'
                }>
                  {connectionStatus === 'connected' ? 'Connected to ROS bridge server' :
                   connectionStatus === 'connecting' ? 'Connecting to ROS bridge server...' :
                   'Disconnected from ROS bridge server'}
                </Alert>
              </Box>
            </CardContent>
          </Card>
        </Grid>
        
        {/* Interface Settings */}
        <Grid item xs={12} md={6}>
          <Card>
            <CardHeader title="Interface Settings" />
            <CardContent>
              <Typography variant="subtitle1" gutterBottom>
                Theme
              </Typography>
              <FormControl fullWidth sx={{ mb: 3 }}>
                <InputLabel id="theme-label">Theme</InputLabel>
                <Select
                  labelId="theme-label"
                  value="light"
                  label="Theme"
                >
                  <MenuItem value="light">Light</MenuItem>
                  <MenuItem value="dark">Dark</MenuItem>
                  <MenuItem value="system">System Default</MenuItem>
                </Select>
              </FormControl>
              
              <Typography variant="subtitle1" gutterBottom>
                Display Settings
              </Typography>
              <FormControlLabel
                control={<Switch defaultChecked />}
                label="Show status bar"
              />
              <FormControlLabel
                control={<Switch defaultChecked />}
                label="Show topic list"
              />
              <FormControlLabel
                control={<Switch defaultChecked />}
                label="Show service list"
              />
            </CardContent>
          </Card>
        </Grid>
        
        {/* Available Topics */}
        <Grid item xs={12} md={6}>
          <Card>
            <CardHeader 
              title="Available Topics" 
              action={
                <TextField
                  label="Filter"
                  variant="outlined"
                  size="small"
                  value={topicFilter}
                  onChange={handleTopicFilterChange}
                />
              }
            />
            <CardContent sx={{ maxHeight: '300px', overflow: 'auto' }}>
              {connectionStatus === 'connected' ? (
                filteredTopics.length > 0 ? (
                  <Box component="ul" sx={{ pl: 2 }}>
                    {filteredTopics.map((topic, index) => (
                      <Box component="li" key={index} sx={{ mb: 1 }}>
                        <Typography variant="body2">{topic}</Typography>
                      </Box>
                    ))}
                  </Box>
                ) : (
                  <Typography variant="body1">No topics available</Typography>
                )
              ) : (
                <Typography variant="body1">Connect to ROS to view available topics</Typography>
              )}
            </CardContent>
          </Card>
        </Grid>
        
        {/* Available Services */}
        <Grid item xs={12} md={6}>
          <Card>
            <CardHeader 
              title="Available Services" 
              action={
                <TextField
                  label="Filter"
                  variant="outlined"
                  size="small"
                  value={serviceFilter}
                  onChange={handleServiceFilterChange}
                />
              }
            />
            <CardContent sx={{ maxHeight: '300px', overflow: 'auto' }}>
              {connectionStatus === 'connected' ? (
                filteredServices.length > 0 ? (
                  <Box component="ul" sx={{ pl: 2 }}>
                    {filteredServices.map((service, index) => (
                      <Box component="li" key={index} sx={{ mb: 1 }}>
                        <Typography variant="body2">{service}</Typography>
                      </Box>
                    ))}
                  </Box>
                ) : (
                  <Typography variant="body1">No services available</Typography>
                )
              ) : (
                <Typography variant="body1">Connect to ROS to view available services</Typography>
              )}
            </CardContent>
          </Card>
        </Grid>
        
        {/* About */}
        <Grid item xs={12}>
          <Card>
            <CardHeader title="About" />
            <CardContent>
              <Typography variant="body1" paragraph>
                Robot Web Interface is a web-based interface for controlling and monitoring ROS-based robots.
                It provides a user-friendly interface for visualizing robot data, controlling the robot,
                and monitoring its status.
              </Typography>
              
              <Divider sx={{ my: 2 }} />
              
              <Grid container spacing={2}>
                <Grid item xs={6} md={3}>
                  <Typography variant="subtitle2">Version</Typography>
                  <Typography variant="body2">1.0.0</Typography>
                </Grid>
                
                <Grid item xs={6} md={3}>
                  <Typography variant="subtitle2">ROS Version</Typography>
                  <Typography variant="body2">ROS 2 Humble</Typography>
                </Grid>
                
                <Grid item xs={6} md={3}>
                  <Typography variant="subtitle2">License</Typography>
                  <Typography variant="body2">MIT</Typography>
                </Grid>
                
                <Grid item xs={6} md={3}>
                  <Typography variant="subtitle2">Author</Typography>
                  <Typography variant="body2">Robot Development Team</Typography>
                </Grid>
              </Grid>
            </CardContent>
          </Card>
        </Grid>
      </Grid>
      
      <Snackbar
        open={snackbarOpen}
        autoHideDuration={6000}
        onClose={handleSnackbarClose}
      >
        <Alert onClose={handleSnackbarClose} severity={snackbarSeverity} sx={{ width: '100%' }}>
          {snackbarMessage}
        </Alert>
      </Snackbar>
    </Box>
  );
};

export default Settings;
