import torch
import numpy as np
from dqn_agent import DQNAgent

# Initialize agent
agent = DQNAgent(input_size=4, num_actions=4)

# Load replay buffer
agent.memory.load('replay_buffer.pkl')
print(f"Loaded replay buffer with {len(agent.memory)} experiences")

# Offline training loop
num_steps = 10000
for step in range(num_steps):
    loss = agent.train_step()
    if loss is not None and step % 100 == 0:
        print(f"Step {step}: Loss = {loss:.4f}")

# Save the trained model
model_path = 'dqn_model_offline.pth'
torch.save(agent.policy_net.state_dict(), model_path)
print(f"Offline training complete. Model saved to {model_path}")