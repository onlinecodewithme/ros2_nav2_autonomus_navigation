import { EventEmitter2 } from 'eventemitter2';

class RosConnection {
  constructor() {
    this.isConnected = false;
    this.apiUrl = 'http://localhost:5000/api';
    this.events = new EventEmitter2();
    this.connectionStatus = 'disconnected'; // 'connected', 'disconnected', 'connecting'
    this.reconnectTimer = null;
    this.reconnectInterval = 5000; // 5 seconds
    this.pollingInterval = 1000; // 1 second
    this.pollingTimers = {};
  }

  connect(url = 'http://localhost:5000/api') {
    if (this.isConnected) {
      this.disconnect();
    }

    this.apiUrl = url;
    this.connectionStatus = 'connecting';
    this.events.emit('status_change', this.connectionStatus);

    // Check if the server is available
    fetch(`${this.apiUrl}/topics`)
      .then(response => {
        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
        return response.json();
      })
      .then(data => {
        console.log('Connected to ROS bridge server');
        this.isConnected = true;
        this.connectionStatus = 'connected';
        this.events.emit('status_change', this.connectionStatus);
        this.events.emit('connected');
        
        if (this.reconnectTimer) {
          clearTimeout(this.reconnectTimer);
          this.reconnectTimer = null;
        }
      })
      .catch(error => {
        console.error('Error connecting to ROS bridge server:', error);
        this.events.emit('error', error);
        
        if (this.connectionStatus === 'connecting') {
          this.connectionStatus = 'disconnected';
          this.events.emit('status_change', this.connectionStatus);
        }
        
        // Attempt to reconnect
        if (!this.reconnectTimer) {
          this.reconnectTimer = setTimeout(() => {
            console.log('Attempting to reconnect to ROS bridge server...');
            this.connect(this.apiUrl);
          }, this.reconnectInterval);
        }
      });
  }

  disconnect() {
    // Clear all polling timers
    Object.keys(this.pollingTimers).forEach(key => {
      clearInterval(this.pollingTimers[key]);
      delete this.pollingTimers[key];
    });
    
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    
    this.isConnected = false;
    this.connectionStatus = 'disconnected';
    this.events.emit('status_change', this.connectionStatus);
    this.events.emit('disconnected');
  }

  // Subscribe to a ROS topic by polling the API
  subscribe(topicName, messageType, callback) {
    if (!this.isConnected) {
      console.error('Cannot subscribe to topic: not connected to ROS');
      return null;
    }

    // Map ROS topic names to API endpoints
    const topicToEndpoint = {
      '/map': 'map',
      '/scan': 'scan',
      '/path': 'path',
      '/robot/battery_state': 'battery',
      // Add more mappings as needed
    };

    const endpoint = topicToEndpoint[topicName] || topicName.replace(/^\//, '');
    
    // Set up polling for this topic
    if (!this.pollingTimers[topicName]) {
      this.pollingTimers[topicName] = setInterval(() => {
        fetch(`${this.apiUrl}/${endpoint}`)
          .then(response => {
            if (!response.ok) {
              if (response.status === 404) {
                // Topic data not available yet, not an error
                return null;
              }
              throw new Error(`HTTP error! status: ${response.status}`);
            }
            return response.json();
          })
          .then(data => {
            if (data) {
              callback(data);
            }
          })
          .catch(error => {
            console.error(`Error polling topic ${topicName}:`, error);
          });
      }, this.pollingInterval);
    }

    return { name: topicName };
  }

  // Unsubscribe from a ROS topic
  unsubscribe(topicName) {
    if (this.pollingTimers[topicName]) {
      clearInterval(this.pollingTimers[topicName]);
      delete this.pollingTimers[topicName];
    }
  }

  // Publish to a ROS topic
  publish(topicName, messageType, message) {
    if (!this.isConnected) {
      console.error('Cannot publish to topic: not connected to ROS');
      return;
    }

    // Map topic names to API endpoints
    const topicToEndpoint = {
      '/cmd_vel': 'cmd_vel',
      // Add more mappings as needed
    };

    const endpoint = topicToEndpoint[topicName] || topicName.replace(/^\//, '');

    // Send POST request to the API
    fetch(`${this.apiUrl}/${endpoint}`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(message),
    })
    .then(response => {
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      return response.json();
    })
    .catch(error => {
      console.error(`Error publishing to topic ${topicName}:`, error);
    });
  }

  // Call a ROS service
  callService(serviceName, serviceType, request, callback) {
    if (!this.isConnected) {
      console.error('Cannot call service: not connected to ROS');
      return;
    }

    // Map service names to API endpoints
    const serviceToEndpoint = {
      '/navigation/navigate_to_goal': 'navigate',
      '/navigation/cancel_navigation': 'cancel_navigation',
      // Add more mappings as needed
    };

    const endpoint = serviceToEndpoint[serviceName] || serviceName.replace(/^\//, '');

    // Send POST request to the API
    fetch(`${this.apiUrl}/${endpoint}`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    })
    .then(response => {
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      return response.json();
    })
    .then(data => {
      if (callback) {
        callback(data);
      }
    })
    .catch(error => {
      console.error(`Error calling service ${serviceName}:`, error);
      if (callback) {
        callback(null, error);
      }
    });
  }

  // Get a list of available topics
  getTopics(callback) {
    if (!this.isConnected) {
      console.error('Cannot get topics: not connected to ROS');
      return;
    }

    fetch(`${this.apiUrl}/topics`)
      .then(response => {
        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
        return response.json();
      })
      .then(data => {
        callback(data);
      })
      .catch(error => {
        console.error('Error getting topics:', error);
      });
  }

  // Get a list of available services
  getServices(callback) {
    if (!this.isConnected) {
      console.error('Cannot get services: not connected to ROS');
      return;
    }

    fetch(`${this.apiUrl}/services`)
      .then(response => {
        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
        return response.json();
      })
      .then(data => {
        callback(data.services);
      })
      .catch(error => {
        console.error('Error getting services:', error);
      });
  }
}

// Create a singleton instance
const rosConnection = new RosConnection();
export default rosConnection;
