import pybullet as p
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import MultivariateNormal
from torch.distributions import kl_divergence

# Define the neural network for the policy
class PolicyNetwork(nn.Module):
    def __init__(self, input_size, output_size):
        super(PolicyNetwork, self).__init__()
        self.fc1 = nn.Linear(input_size, 64)
        self.fc2 = nn.Linear(64, output_size)
        
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        mean = torch.tanh(self.fc2(x))
        return mean

# Hyperparameters
num_epochs = 1000
num_steps_per_epoch = 1000
num_robot_joints = 7
state_dim = num_robot_joints
action_dim = num_robot_joints
learning_rate = 0.0003
clip_epsilon = 0.2
intrins_reward_weight = 0.01

# Initialize PyBullet simulation
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("path_to_your_robot.urdf", [0, 0, 1])

# Initialize policy network and optimizer
policy_net = PolicyNetwork(state_dim, action_dim)
optimizer = optim.Adam(policy_net.parameters(), lr=learning_rate)

# Training loop
for epoch in range(num_epochs):
    for _ in range(num_steps_per_epoch):
        # Sample a state (current joint angles)
        state = np.random.uniform(-1, 1, size=(num_robot_joints,))
        
        # Convert state to PyTorch tensor
        state_tensor = torch.tensor(state, dtype=torch.float32)
        
        # Forward pass through the policy network
        action_mean = policy_net(state_tensor)
        dist = MultivariateNormal(action_mean, torch.diag(torch.ones(action_dim)))
        
        # Sample an action from the distribution
        action = dist.sample()
        
        # Calculate intrinsic motivation reward
        prev_state_tensor = state_tensor  # In a real scenario, store the previous state
        intrinsic_reward = -torch.norm(state_tensor - prev_state_tensor)
        
        # Get log probability of the action under the distribution
        log_prob = dist.log_prob(action).sum()
        
        # Perform an action in the environment
        action_np = action.detach().numpy()
        for joint_index, joint_value in enumerate(action_np):
            p.setJointMotorControl2(robotId, joint_index, p.POSITION_CONTROL, targetPosition=joint_value)
        p.stepSimulation()
        
        # Calculate extrinsic reward (distance to a target, for example)
        target_pos = np.array([0.5, 0.5, 1.0])
        end_effector_pos, _ = p.getLinkState(robotId, end_effector_link_index)[:2]
        extrinsic_reward = -np.linalg.norm(end_effector_pos - target_pos)
        
        # Calculate total reward
        total_reward = extrinsic_reward + intrins_reward_weight * intrinsic_reward.item()
        
        # Update policy using PPO
        ratio = torch.exp(log_prob - old_log_prob)
        surrogate1 = ratio * adv
        surrogate2 = torch.clamp(ratio, 1 - clip_epsilon, 1 + clip_epsilon) * adv
        surrogate_loss = -torch.min(surrogate1, surrogate2).mean()
        
        # Calculate KL divergence
        kl = kl_divergence(dist, old_dist).mean()
        
        # PPO loss function
        loss = surrogate_loss + 0.1 * kl
        
        # Optimize the policy network
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        # Update old distribution for next iteration
        old_dist = dist
        old_log_prob = log_prob.detach()
    
    print(f"Epoch {epoch+1}/{num_epochs} - Total Reward: {total_reward:.2f}")
    
# Disconnect PyBullet simulation
p.disconnect()
