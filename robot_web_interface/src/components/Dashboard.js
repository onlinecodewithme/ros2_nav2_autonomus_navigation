import React, { useState, useEffect } from 'react';
import { 
  Grid, 
  Paper, 
  Typography, 
  Box, 
  Card, 
  CardContent, 
  CardHeader,
  List,
  ListItem,
  ListItemText,
  Divider
} from '@mui/material';
import rosConnection from '../utils/rosConnection';

const Dashboard = () => {
  const [robotStatus, setRobotStatus] = useState({
    battery: 0,
    cpuUsage: 0,
    memoryUsage: 0,
    temperature: 0,
    uptime: 0
  });
  
  const [topicList, setTopicList] = useState([]);
  const [serviceList, setServiceList] = useState([]);

  useEffect(() => {
    // Subscribe to robot status topics when connected
    const handleConnection = () => {
      // Example subscription to battery status
      rosConnection.subscribe(
        '/robot/battery_state',
        'sensor_msgs/BatteryState',
        (message) => {
          setRobotStatus(prev => ({
            ...prev,
            battery: message.percentage
          }));
        }
      );
      
      // Get available topics and services
      rosConnection.getTopics((topics) => {
        setTopicList(topics.topics.slice(0, 10)); // Show first 10 topics
      });
      
      rosConnection.getServices((services) => {
        setServiceList(services.slice(0, 10)); // Show first 10 services
      });
    };

    // Set up event listeners
    rosConnection.events.on('connected', handleConnection);
    
    // Clean up subscriptions
    return () => {
      rosConnection.events.off('connected', handleConnection);
      rosConnection.unsubscribe('/robot/battery_state');
    };
  }, []);

  // Format uptime in hours:minutes:seconds
  const formatUptime = (seconds) => {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = Math.floor(seconds % 60);
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };

  return (
    <Box sx={{ flexGrow: 1, mt: 2 }}>
      <Typography variant="h4" gutterBottom>
        Dashboard
      </Typography>
      
      <Grid container spacing={3}>
        {/* System Status */}
        <Grid item xs={12} md={6}>
          <Card>
            <CardHeader title="System Status" />
            <CardContent>
              <Grid container spacing={2}>
                <Grid item xs={6}>
                  <Paper elevation={0} sx={{ p: 2, bgcolor: 'background.default' }}>
                    <Typography variant="body2" color="text.secondary">
                      Battery
                    </Typography>
                    <Typography variant="h6">
                      {robotStatus.battery.toFixed(1)}%
                    </Typography>
                  </Paper>
                </Grid>
                <Grid item xs={6}>
                  <Paper elevation={0} sx={{ p: 2, bgcolor: 'background.default' }}>
                    <Typography variant="body2" color="text.secondary">
                      CPU Usage
                    </Typography>
                    <Typography variant="h6">
                      {robotStatus.cpuUsage.toFixed(1)}%
                    </Typography>
                  </Paper>
                </Grid>
                <Grid item xs={6}>
                  <Paper elevation={0} sx={{ p: 2, bgcolor: 'background.default' }}>
                    <Typography variant="body2" color="text.secondary">
                      Memory Usage
                    </Typography>
                    <Typography variant="h6">
                      {robotStatus.memoryUsage.toFixed(1)}%
                    </Typography>
                  </Paper>
                </Grid>
                <Grid item xs={6}>
                  <Paper elevation={0} sx={{ p: 2, bgcolor: 'background.default' }}>
                    <Typography variant="body2" color="text.secondary">
                      Temperature
                    </Typography>
                    <Typography variant="h6">
                      {robotStatus.temperature.toFixed(1)}°C
                    </Typography>
                  </Paper>
                </Grid>
                <Grid item xs={12}>
                  <Paper elevation={0} sx={{ p: 2, bgcolor: 'background.default' }}>
                    <Typography variant="body2" color="text.secondary">
                      Uptime
                    </Typography>
                    <Typography variant="h6">
                      {formatUptime(robotStatus.uptime)}
                    </Typography>
                  </Paper>
                </Grid>
              </Grid>
            </CardContent>
          </Card>
        </Grid>
        
        {/* ROS Information */}
        <Grid item xs={12} md={6}>
          <Card>
            <CardHeader title="ROS Information" />
            <CardContent>
              <Typography variant="subtitle1" gutterBottom>
                Available Topics
              </Typography>
              <List dense>
                {topicList.length > 0 ? (
                  topicList.map((topic, index) => (
                    <React.Fragment key={index}>
                      <ListItem>
                        <ListItemText primary={topic} />
                      </ListItem>
                      {index < topicList.length - 1 && <Divider />}
                    </React.Fragment>
                  ))
                ) : (
                  <ListItem>
                    <ListItemText primary="No topics available" />
                  </ListItem>
                )}
              </List>
              
              <Typography variant="subtitle1" gutterBottom sx={{ mt: 2 }}>
                Available Services
              </Typography>
              <List dense>
                {serviceList.length > 0 ? (
                  serviceList.map((service, index) => (
                    <React.Fragment key={index}>
                      <ListItem>
                        <ListItemText primary={service} />
                      </ListItem>
                      {index < serviceList.length - 1 && <Divider />}
                    </React.Fragment>
                  ))
                ) : (
                  <ListItem>
                    <ListItemText primary="No services available" />
                  </ListItem>
                )}
              </List>
            </CardContent>
          </Card>
        </Grid>
        
        {/* Recent Activity */}
        <Grid item xs={12}>
          <Card>
            <CardHeader title="Recent Activity" />
            <CardContent>
              <Typography variant="body1">
                No recent activity to display.
              </Typography>
            </CardContent>
          </Card>
        </Grid>
      </Grid>
    </Box>
  );
};

export default Dashboard;
