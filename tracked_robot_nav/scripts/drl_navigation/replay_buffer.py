#!/usr/bin/env python3

import numpy as np

class ReplayBuffer:
    """
    A simple replay buffer for reinforcement learning.
    """
    def __init__(self, max_size=1e6, seed=0):
        """
        Initialize a replay buffer.
        
        Args:
            max_size (int): Maximum size of the buffer
            seed (int): Random seed for reproducibility
        """
        self.storage = []
        self.max_size = max_size
        self.ptr = 0
        np.random.seed(seed)
    
    def add(self, state, action, reward, done, next_state):
        """
        Add a transition to the buffer.
        
        Args:
            state: Current state
            action: Action taken
            reward: Reward received
            done: Whether the episode is done
            next_state: Next state
        """
        if len(self.storage) == self.max_size:
            self.storage[int(self.ptr)] = (state, action, reward, done, next_state)
            self.ptr = (self.ptr + 1) % self.max_size
        else:
            self.storage.append((state, action, reward, done, next_state))
    
    def sample_batch(self, batch_size=100):
        """
        Sample a batch of transitions from the buffer.
        
        Args:
            batch_size (int): Size of the batch
            
        Returns:
            tuple: Batch of (states, actions, rewards, dones, next_states)
        """
        ind = np.random.randint(0, len(self.storage), size=batch_size)
        states, actions, rewards, dones, next_states = [], [], [], [], []
        
        for i in ind:
            s, a, r, d, s_ = self.storage[i]
            states.append(np.array(s, copy=False))
            actions.append(np.array(a, copy=False))
            rewards.append(np.array(r, copy=False))
            dones.append(np.array(d, copy=False))
            next_states.append(np.array(s_, copy=False))
        
        return (
            np.array(states),
            np.array(actions),
            np.array(rewards).reshape(-1, 1),
            np.array(dones).reshape(-1, 1),
            np.array(next_states),
        )
