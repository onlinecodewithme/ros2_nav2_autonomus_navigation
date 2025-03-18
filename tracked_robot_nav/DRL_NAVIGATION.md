# DRL Navigation Technical Documentation

This document provides detailed technical information about the Deep Reinforcement Learning (DRL) navigation implementation in the tracked_robot_nav project.

## Table of Contents

1. [System Architecture](#system-architecture)
2. [TD3 Algorithm Implementation](#td3-algorithm-implementation)
3. [Neural Network Architecture](#neural-network-architecture)
4. [State and Action Spaces](#state-and-action-spaces)
5. [Reward Function Design](#reward-function-design)
6. [ZED Camera Integration](#zed-camera-integration)
7. [ROS Integration](#ros-integration)
8. [Training Process](#training-process)
9. [Hyperparameter Tuning](#hyperparameter-tuning)
10. [Performance Evaluation](#performance-evaluation)
11. [Code Structure](#code-structure)

## System Architecture

The DRL navigation system follows a modular architecture with the following components:

```
                  ┌─────────────────┐
                  │  ROS Environment│
                  └────────┬────────┘
                           │
                           ▼
┌──────────────┐    ┌─────────────┐    ┌───────────────┐
│ ZED Camera   │───▶│ Environment │◀───│ Goal Setting  │
│ Interface    │    │ Interface   │    │ Interface     │
└──────────────┘    └──────┬──────┘    └───────────────┘
                           │
                           ▼
                  ┌─────────────────┐
                  │  TD3 Agent      │
                  └────────┬────────┘
                           │
                  ┌────────┴────────┐
                  │                 │
         ┌────────▼─────┐   ┌───────▼────────┐
         │ Actor Network│   │ Critic Networks│
         └──────────────┘   └────────────────┘
                  │                 │
                  └────────┬────────┘
                           │
                           ▼
                  ┌─────────────────┐
                  │  Replay Buffer  │
                  └─────────────────┘
```

### Component Responsibilities

- **ZED Camera Interface**: Processes depth and point cloud data from the ZED camera
- **Environment Interface**: Translates ROS observations into agent states and agent actions into robot commands
- **Goal Setting Interface**: Manages navigation goals and provides relative goal information to the agent
- **TD3 Agent**: Implements the TD3 algorithm for learning and decision making
- **Actor Network**: Learns the policy mapping states to actions
- **Critic Networks**: Learn to estimate the Q-value (expected return) of state-action pairs
- **Replay Buffer**: Stores experiences for off-policy learning

## TD3 Algorithm Implementation

The Twin Delayed Deep Deterministic Policy Gradient (TD3) algorithm is implemented in `td3_zed_navigation.py`. Key features include:

### 1. Twin Critics

Two critic networks are used to reduce overestimation bias:

```python
# Critic networks initialization
self.critic1 = Critic(state_dim, action_dim).to(device)
self.critic2 = Critic(state_dim, action_dim).to(device)
self.critic1_target = Critic(state_dim, action_dim).to(device)
self.critic2_target = Critic(state_dim, action_dim).to(device)
```

### 2. Delayed Policy Updates

The policy is updated less frequently than the critics to allow the critics to converge:

```python
# Update critics
critic_loss = self._update_critics(batch)
critic_info = {"critic_loss": critic_loss}

# Delayed policy updates
if self.total_it % self.policy_freq == 0:
    actor_loss = self._update_actor(batch)
    self._update_targets()
    actor_info = {"actor_loss": actor_loss}
```

### 3. Target Policy Smoothing

Noise is added to target actions to smooth the value estimate:

```python
# Target policy smoothing
noise = torch.randn_like(action) * self.policy_noise
noise = noise.clamp(-self.noise_clip, self.noise_clip)
next_action = (self.actor_target(next_state) + noise).clamp(-1, 1)
```

### 4. Soft Target Updates

Target networks are updated using soft updates to improve stability:

```python
def _update_targets(self):
    # Soft update of target networks
    for param, target_param in zip(self.critic1.parameters(), self.critic1_target.parameters()):
        target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
    
    for param, target_param in zip(self.critic2.parameters(), self.critic2_target.parameters()):
        target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
    
    for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
        target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
```

## Neural Network Architecture

### Actor Network

The actor network maps states to actions using a feedforward architecture:

```python
class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(Actor, self).__init__()
        
        self.l1 = nn.Linear(state_dim, hidden_dim)
        self.l2 = nn.Linear(hidden_dim, hidden_dim)
        self.l3 = nn.Linear(hidden_dim, action_dim)
        
    def forward(self, state):
        a = F.relu(self.l1(state))
        a = F.relu(self.l2(a))
        return torch.tanh(self.l3(a))  # Output in range [-1, 1]
```

### Critic Network

Each critic network estimates Q-values for state-action pairs:

```python
class Critic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(Critic, self).__init__()
        
        # Q1 architecture
        self.l1 = nn.Linear(state_dim + action_dim, hidden_dim)
        self.l2 = nn.Linear(hidden_dim, hidden_dim)
        self.l3 = nn.Linear(hidden_dim, 1)
        
    def forward(self, state, action):
        sa = torch.cat([state, action], 1)
        
        q = F.relu(self.l1(sa))
        q = F.relu(self.l2(q))
        q = self.l3(q)
        
        return q
```

## State and Action Spaces

### State Space

The state representation is a combination of:

1. **Point Cloud Features**: Processed from ZED camera data
   - Distance to nearest obstacles in different angular sectors
   - Height variations in the terrain
   - Density of points in each sector

2. **Robot State**:
   - Current linear and angular velocities
   - Battery level (optional)

3. **Goal Information**:
   - Relative distance to goal
   - Relative angle to goal

The state vector has a total dimension of 64 (configurable).

### Action Space

The action space is continuous and consists of:

1. **Linear Velocity**: Controls forward/backward movement (-1.0 to 1.0, scaled to actual robot limits)
2. **Angular Velocity**: Controls turning left/right (-1.0 to 1.0, scaled to actual robot limits)

## Reward Function Design

The reward function is designed to encourage efficient navigation to the goal while avoiding obstacles:

```python
def compute_reward(self, state, action, next_state, done, info):
    reward = 0.0
    
    # Extract relevant information
    goal_distance = state['goal_distance']
    next_goal_distance = next_state['goal_distance']
    collision = info.get('collision', False)
    goal_reached = info.get('goal_reached', False)
    
    # Distance reduction reward
    distance_reduction = goal_distance - next_goal_distance
    reward += self.distance_reward_scale * distance_reduction
    
    # Collision penalty
    if collision:
        reward += self.collision_penalty
        done = True
    
    # Goal reached bonus
    if goal_reached:
        reward += self.goal_reward
        done = True
    
    # Smoothness penalty (penalize large changes in action)
    if self.last_action is not None:
        action_diff = np.linalg.norm(action - self.last_action)
        reward -= self.smoothness_penalty * action_diff
    
    self.last_action = action.copy()
    
    # Living penalty (encourages faster completion)
    reward -= self.living_penalty
    
    return reward, done
```

Key reward components:

- **Progress Reward**: Positive reward for moving closer to the goal
- **Collision Penalty**: Large negative reward for colliding with obstacles
- **Goal Reward**: Large positive reward for reaching the goal
- **Smoothness Penalty**: Small negative reward for jerky movements
- **Living Penalty**: Small negative reward for each step (encourages efficiency)

## ZED Camera Integration

The ZED camera provides depth and point cloud data that is processed to create a compact state representation:

### Point Cloud Processing

```python
def process_point_cloud(self, point_cloud_msg):
    # Convert ROS PointCloud2 to numpy array
    pc_array = point_cloud2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
    pc_array = np.array(list(pc_array))
    
    if pc_array.shape[0] == 0:
        return np.zeros(self.num_sectors * 2)  # Empty point cloud
    
    # Filter points within relevant range
    mask = (pc_array[:, 0] > 0) & (pc_array[:, 0] < self.max_range) & \
           (np.abs(pc_array[:, 1]) < self.max_range) & \
           (np.abs(pc_array[:, 2]) < 1.0)  # Height filter
    pc_filtered = pc_array[mask]
    
    if pc_filtered.shape[0] == 0:
        return np.zeros(self.num_sectors * 2)  # No points after filtering
    
    # Compute angles and distances
    angles = np.arctan2(pc_filtered[:, 1], pc_filtered[:, 0])
    distances = np.sqrt(pc_filtered[:, 0]**2 + pc_filtered[:, 1]**2)
    
    # Divide into sectors
    sector_size = 2 * np.pi / self.num_sectors
    sector_features = np.zeros(self.num_sectors * 2)
    
    for i in range(self.num_sectors):
        sector_min = -np.pi + i * sector_size
        sector_max = sector_min + sector_size
        sector_mask = (angles >= sector_min) & (angles < sector_max)
        sector_points = distances[sector_mask]
        
        if sector_points.size > 0:
            # Minimum distance in this sector
            sector_features[i] = np.min(sector_points) / self.max_range
            # Point density in this sector
            sector_features[i + self.num_sectors] = min(1.0, sector_points.size / 100)
        else:
            sector_features[i] = 1.0  # No obstacles detected
            sector_features[i + self.num_sectors] = 0.0  # No points
    
    return sector_features
```

## ROS Integration

The DRL navigation system integrates with ROS through:

### 1. Subscribers

```python
# Point cloud subscription
self.point_cloud_sub = rospy.Subscriber(
    '/zed/point_cloud/cloud_registered',
    PointCloud2,
    self.point_cloud_callback,
    queue_size=1
)

# Odometry subscription
self.odom_sub = rospy.Subscriber(
    '/odom',
    Odometry,
    self.odom_callback,
    queue_size=1
)

# Goal subscription
self.goal_sub = rospy.Subscriber(
    '/goal_pose',
    PoseStamped,
    self.goal_callback,
    queue_size=1
)
```

### 2. Publishers

```python
# Command velocity publisher
self.cmd_vel_pub = rospy.Publisher(
    '/cmd_vel',
    Twist,
    queue_size=1
)

# Visualization publishers
self.action_pub = rospy.Publisher(
    '/drl_navigation/action',
    Float32MultiArray,
    queue_size=1
)

self.state_pub = rospy.Publisher(
    '/drl_navigation/state',
    Float32MultiArray,
    queue_size=1
)
```

### 3. TF Integration

```python
def get_robot_pose(self):
    try:
        transform = self.tf_buffer.lookup_transform(
            'map',
            'base_link',
            rospy.Time(0),
            rospy.Duration(1.0)
        )
        
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        
        # Extract yaw from quaternion
        orientation = transform.transform.rotation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
        return np.array([x, y, yaw])
    
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"TF lookup failed: {e}")
        return None
```

## Training Process

The training process is implemented in `run_drl_navigation.py`:

```python
def train(args):
    # Initialize ROS node
    rospy.init_node('drl_navigation_training', anonymous=True)
    
    # Create environment
    env = NavigationEnvironment()
    
    # Initialize agent
    state_dim = env.get_state_dim()
    action_dim = env.get_action_dim()
    max_action = env.get_max_action()
    
    agent = TD3(state_dim, action_dim, max_action, args)
    
    # Load existing model if specified
    if args.model_path:
        agent.load(args.model_path)
        print(f"Loaded model from {args.model_path}")
    
    # Initialize replay buffer
    replay_buffer = ReplayBuffer(state_dim, action_dim, args.buffer_size)
    
    # Training loop
    total_timesteps = 0
    episode_num = 0
    
    while total_timesteps < args.max_timesteps:
        episode_reward = 0
        episode_steps = 0
        done = False
        state = env.reset()
        
        while not done and episode_steps < args.max_episode_steps:
            # Select action with exploration noise
            if total_timesteps < args.start_timesteps:
                action = env.sample_action()
            else:
                action = agent.select_action(state)
                action = action + np.random.normal(0, args.exploration_noise, size=action_dim)
                action = np.clip(action, -max_action, max_action)
            
            # Execute action
            next_state, reward, done, info = env.step(action)
            
            # Store experience in replay buffer
            replay_buffer.add(state, action, next_state, reward, done)
            
            # Update agent
            if total_timesteps >= args.start_timesteps:
                agent.update(replay_buffer, args.batch_size)
            
            state = next_state
            episode_reward += reward
            episode_steps += 1
            total_timesteps += 1
            
            # Evaluation
            if total_timesteps % args.eval_freq == 0:
                evaluate_policy(agent, env, args.eval_episodes)
                
                # Save model
                if args.save_path:
                    agent.save(args.save_path)
                    print(f"Saved model to {args.save_path}")
        
        # Log episode results
        print(f"Episode {episode_num}: {episode_reward:.2f} reward, {episode_steps} steps")
        episode_num += 1
```

## Hyperparameter Tuning

Key hyperparameters and their effects:

| Parameter | Default Value | Effect |
|-----------|---------------|--------|
| Learning Rate | 0.0003 | Controls how quickly the networks adapt |
| Gamma | 0.99 | Discount factor for future rewards |
| Tau | 0.005 | Soft update coefficient for target networks |
| Policy Noise | 0.2 | Standard deviation of noise added to target actions |
| Noise Clip | 0.5 | Maximum value of target policy noise |
| Policy Frequency | 2 | Number of critic updates per actor update |
| Batch Size | 256 | Number of experiences used for each update |
| Buffer Size | 1,000,000 | Maximum number of experiences stored |
| Exploration Noise | 0.1 | Standard deviation of noise added during exploration |

## Performance Evaluation

The DRL navigation system is evaluated using the following metrics:

1. **Success Rate**: Percentage of episodes where the goal is reached
2. **Average Return**: Mean cumulative reward per episode
3. **Average Episode Length**: Mean number of steps per episode
4. **Collision Rate**: Percentage of episodes ending in collision
5. **Path Efficiency**: Ratio of optimal path length to actual path length

Evaluation is performed in both simulation and real-world environments.

## Code Structure

The DRL navigation code is organized as follows:

```
scripts/drl_navigation/
├── __init__.py                  # Package initialization
├── td3_zed_navigation.py        # TD3 algorithm implementation
├── replay_buffer.py             # Experience replay buffer
├── run_drl_navigation.py        # Training script
├── test_drl_navigation.py       # Testing script
├── point_cloud2.py              # Point cloud processing utilities
└── utils.py                     # Miscellaneous utility functions

config/
└── drl_navigation_params.yaml   # Configuration parameters

models/
└── td3_navigation_model.pt      # Pre-trained model weights

logs/
└── drl_training_latest.log      # Training logs
```

### Key Classes

1. **TD3**: Implements the TD3 algorithm
2. **Actor**: Neural network for policy learning
3. **Critic**: Neural network for value estimation
4. **ReplayBuffer**: Stores and samples experiences
5. **NavigationEnvironment**: Interfaces with ROS and the robot
6. **PointCloudProcessor**: Processes ZED camera point cloud data

## References

1. Fujimoto, S., van Hoof, H., & Meger, D. (2018). Addressing Function Approximation Error in Actor-Critic Methods. arXiv:1802.09477.
2. Lillicrap, T. P., Hunt, J. J., Pritzel, A., Heess, N., Erez, T., Tassa, Y., Silver, D., & Wierstra, D. (2015). Continuous control with deep reinforcement learning. arXiv:1509.02971.
3. Schulman, J., Wolski, F., Dhariwal, P., Radford, A., & Klimov, O. (2017). Proximal Policy Optimization Algorithms. arXiv:1707.06347.
4. Tai, L., Paolo, G., & Liu, M. (2017). Virtual-to-real deep reinforcement learning: Continuous control of mobile robots for mapless navigation. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS).
5. Xie, L., Wang, S., Rosa, S., Markham, A., & Trigoni, N. (2018). Learning with Training Wheels: Speeding up Training with a Simple Controller for Deep Reinforcement Learning. IEEE International Conference on Robotics and Automation (ICRA).
