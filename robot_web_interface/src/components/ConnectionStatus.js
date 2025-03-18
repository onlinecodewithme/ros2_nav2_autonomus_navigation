import React from 'react';
import { Box, Typography } from '@mui/material';

const ConnectionStatus = ({ status }) => {
  let statusClass = '';
  let statusText = '';

  switch (status) {
    case 'connected':
      statusClass = 'status-connected';
      statusText = 'Connected';
      break;
    case 'connecting':
      statusClass = 'status-connecting';
      statusText = 'Connecting...';
      break;
    case 'disconnected':
    default:
      statusClass = 'status-disconnected';
      statusText = 'Disconnected';
      break;
  }

  return (
    <Box className="connection-status">
      <div className={`status-indicator ${statusClass}`}></div>
      <Typography variant="body2">{statusText}</Typography>
    </Box>
  );
};

export default ConnectionStatus;
