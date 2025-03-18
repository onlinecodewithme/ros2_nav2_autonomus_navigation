import React, { useState, useEffect } from 'react';
import { Routes, Route, Link } from 'react-router-dom';
import { 
  AppBar, 
  Toolbar, 
  Typography, 
  Container, 
  Box, 
  IconButton, 
  Drawer, 
  List, 
  ListItem, 
  ListItemIcon, 
  ListItemText,
  Divider
} from '@mui/material';
import MenuIcon from '@mui/icons-material/Menu';
import HomeIcon from '@mui/icons-material/Home';
import MapIcon from '@mui/icons-material/Map';
import SettingsIcon from '@mui/icons-material/Settings';
import SportsEsportsIcon from '@mui/icons-material/SportsEsports';
import InfoIcon from '@mui/icons-material/Info';

import Dashboard from './Dashboard';
import RobotControl from './RobotControl';
import MapVisualization from './MapVisualization';
import Settings from './Settings';
import ConnectionStatus from './ConnectionStatus';
import rosConnection from '../utils/rosConnection';

const App = () => {
  const [drawerOpen, setDrawerOpen] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('disconnected');

  useEffect(() => {
    // Initialize ROS connection
    rosConnection.connect();

    // Listen for connection status changes
    const handleStatusChange = (status) => {
      setConnectionStatus(status);
    };

    rosConnection.events.on('status_change', handleStatusChange);

    // Cleanup on component unmount
    return () => {
      rosConnection.events.off('status_change', handleStatusChange);
      rosConnection.disconnect();
    };
  }, []);

  const toggleDrawer = (open) => (event) => {
    if (
      event.type === 'keydown' &&
      (event.key === 'Tab' || event.key === 'Shift')
    ) {
      return;
    }
    setDrawerOpen(open);
  };

  const menuItems = [
    { text: 'Dashboard', icon: <HomeIcon />, path: '/' },
    { text: 'Robot Control', icon: <SportsEsportsIcon />, path: '/control' },
    { text: 'Map Visualization', icon: <MapIcon />, path: '/map' },
    { text: 'Settings', icon: <SettingsIcon />, path: '/settings' },
  ];

  const drawer = (
    <Box
      sx={{ width: 250 }}
      role="presentation"
      onClick={toggleDrawer(false)}
      onKeyDown={toggleDrawer(false)}
    >
      <Box sx={{ p: 2 }}>
        <Typography variant="h6" component="div">
          Robot Interface
        </Typography>
      </Box>
      <Divider />
      <List>
        {menuItems.map((item) => (
          <ListItem button key={item.text} component={Link} to={item.path}>
            <ListItemIcon>{item.icon}</ListItemIcon>
            <ListItemText primary={item.text} />
          </ListItem>
        ))}
      </List>
      <Divider />
      <Box sx={{ p: 2 }}>
        <ConnectionStatus status={connectionStatus} />
      </Box>
    </Box>
  );

  return (
    <div className="app-container">
      <AppBar position="static">
        <Toolbar>
          <IconButton
            size="large"
            edge="start"
            color="inherit"
            aria-label="menu"
            sx={{ mr: 2 }}
            onClick={toggleDrawer(true)}
          >
            <MenuIcon />
          </IconButton>
          <Typography variant="h6" component="div" sx={{ flexGrow: 1 }}>
            Robot Control & Visualization
          </Typography>
          <Box sx={{ display: 'flex', alignItems: 'center' }}>
            <ConnectionStatus status={connectionStatus} />
          </Box>
        </Toolbar>
      </AppBar>
      
      <Drawer
        anchor="left"
        open={drawerOpen}
        onClose={toggleDrawer(false)}
      >
        {drawer}
      </Drawer>
      
      <Container className="content-container">
        <Routes>
          <Route path="/" element={<Dashboard />} />
          <Route path="/control" element={<RobotControl />} />
          <Route path="/map" element={<MapVisualization />} />
          <Route path="/settings" element={<Settings />} />
        </Routes>
      </Container>
    </div>
  );
};

export default App;
