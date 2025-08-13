import torch
import torch.nn as nn
import torch.optim as optim
import random
import numpy as np
from collections import deque
import pickle  # Added for save/load

class DQN(nn.Module):
    def __init__(self, input_size, num_actions):
        super().__init__()
        # Force input_size to 5 for new obs (fire_or_no, fire_size, lidar_min_distance, angle_to_fire, bbox_x)
        self.net = nn.Sequential(
            nn.Linear(8, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, num_actions)
        )
    
    def forward(self, x):
        return self.net(x)

class ReplayBuffer:
    def __init__(self, capacity):
        self.buffer = deque(maxlen=capacity)
    
    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))
    
    def sample(self, batch_size):
        if len(self.buffer) < batch_size:
            return None
        batch = random.sample(self.buffer, batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        return np.array(states), actions, rewards, np.array(next_states), dones
    
    def __len__(self):
        return len(self.buffer)
    
    # Added for offline collection
    def save(self, path):
        with open(path, 'wb') as f:
            pickle.dump(list(self.buffer), f)
    
    def load(self, path):
        with open(path, 'rb') as f:
            data = pickle.load(f)
        self.buffer = deque(data, maxlen=self.capacity)

class DQNAgent:
    def __init__(self, input_size, num_actions):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy_net = DQN(input_size, num_actions).to(self.device)
        self.target_net = DQN(input_size, num_actions).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=1e-4)
        self.memory = ReplayBuffer(10000)
        self.batch_size = 32
        self.gamma = 0.99
        self.update_steps = 0
        self.num_actions = num_actions
        self.loss_history = []
        self.reward_history = []

    def select_action(self, state, epsilon):
        if random.random() < epsilon:
            return random.randint(0, self.num_actions - 1)
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        with torch.no_grad():
            q_values = self.policy_net(state_tensor)
        return q_values.argmax().item()
    
    def train_step(self):
        if len(self.memory) < self.batch_size:
            return None
        
        batch = self.memory.sample(self.batch_size)
        if batch is None:
            return None
            
        states, actions, rewards, next_states, dones = batch
        
        states = torch.FloatTensor(states).to(self.device)
        actions = torch.LongTensor(actions).unsqueeze(1).to(self.device)
        rewards = torch.FloatTensor(rewards).unsqueeze(1).to(self.device)
        next_states = torch.FloatTensor(next_states).to(self.device)
        dones = torch.FloatTensor(dones).unsqueeze(1).to(self.device)
        
        current_q = self.policy_net(states).gather(1, actions)
        next_q = self.target_net(next_states).max(1)[0].unsqueeze(1)
        target_q = rewards + (1 - dones) * self.gamma * next_q
        
        loss = nn.MSELoss()(current_q, target_q.detach())
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        self.update_steps += 1
        if self.update_steps % 100 == 0:
            self.target_net.load_state_dict(self.policy_net.state_dict())
        
        self.loss_history.append(loss.item())
        self.reward_history.append(float(rewards.mean().item()))
        return loss.item()