import torch
import torch.nn as nn 
import torch.nn.functional as F
import torch.optim as optim
import random
import numpy as np

from models import Critic, Actor
from common.replay_buffers import BasicBuffer


class TD3Agent:

    def __init__(self, env, gamma, tau, buffer_maxlen, delay_step, noise_std, noise_bound, critic_lr, actor_lr):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        self.env = env
        self.obs_dim = env.observation_space.shape[0]
        self.action_dim = env.action_space.shape[0]

        # hyperparameters    
        self.gamma = gamma
        self.tau = tau
        self.noise_std = noise_std
        self.noise_bound = noise_bound
        self.update_step = 0 
        self.delay_step = delay_step
        
        # initialize actor and critic networks
        self.critic1 = Critic(self.obs_dim, self.action_dim).to(self.device)
        self.critic2 = Critic(self.obs_dim, self.action_dim).to(self.device)
        self.critic1_target = Critic(self.obs_dim, self.action_dim).to(self.device)
        self.critic2_target = Critic(self.obs_dim, self.action_dim).to(self.device)
        
        self.actor = Actor(self.obs_dim, self.action_dim).to(self.device)
        self.actor_target = Actor(self.obs_dim, self.action_dim).to(self.device)
    
        # Copy critic target parameters
        for target_param, param in zip(self.critic1_target.parameters(), self.critic1.parameters()):
            target_param.data.copy_(param.data)

        for target_param, param in zip(self.critic2_target.parameters(), self.critic2.parameters()):
            target_param.data.copy_(param.data)

        # initialize optimizers        
        self.critic1_optimizer = optim.Adam(self.critic1.parameters(), lr=critic_lr)
        self.critic2_optimizer = optim.Adam(self.critic1.parameters(), lr=critic_lr) 
        self.actor_optimizer  = optim.Adam(self.actor.parameters(), lr=actor_lr)

        # self.critic1.load_state_dict(torch.load("./critic1.pt"))
        # self.critic2.load_state_dict(torch.load("./critic2.pt"))
        # self.critic1_target.load_state_dict(torch.load("./critic1_target.pt"))
        # self.critic2_target.load_state_dict(torch.load("./critic2_target.pt"))
        # self.actor.load_state_dict(torch.load("./actor.pt"))
        # self.actor_target.load_state_dict(torch.load("./actor_target.pt"))   
        
        self.replay_buffer = BasicBuffer(buffer_maxlen)        

    def get_action(self, obs):
        action = []
        # print("obs")
        # print(obs)
        if random.random() > 0.3:
            action = self.actor.forward(obs)
  
            #action = action.squeeze(0).cpu().detach().numpy()
        else:
            print("taking random action lol")
            #action = np.array([random.random() for i in range(4)])
            action = np.array([random.random() for i in range(4)])

            # action = torch.FloatTensor(array)
            # print(array)
        return action
    
    def update(self, batch_size):
        state_batch, action_batch, reward_batch, next_state_batch, masks = self.replay_buffer.sample(batch_size)

        print(action_batch)


        action_space_noise = self.generate_action_space_noise(action_batch)

        #print(action_space_noise)
        next_actions = self.actor.forward(state_batch) #+ action_space_noise
        next_Q1 = self.critic1_target.forward(next_state_batch, next_actions)
        next_Q2 = self.critic2_target.forward(next_state_batch, next_actions)
        expected_Q = reward_batch + self.gamma * torch.min(next_Q1, next_Q2)

        # critic loss
        curr_Q1 = self.critic1.forward(state_batch, action_batch)
        curr_Q2 = self.critic2.forward(state_batch, action_batch)
        critic1_loss = F.mse_loss(curr_Q1, expected_Q.detach())
        critic2_loss = F.mse_loss(curr_Q2, expected_Q.detach())
        
        # update critics
        self.critic1_optimizer.zero_grad()
        critic1_loss.backward()
        self.critic1_optimizer.step()

        self.critic2_optimizer.zero_grad()
        critic2_loss.backward()
        self.critic2_optimizer.step()

        # delyaed update for actor & target networks  
        if(self.update_step % self.delay_step == 0):
            # actor
            self.actor_optimizer.zero_grad()
            policy_gradient = -self.critic1(state_batch, self.actor(state_batch)).mean()
            policy_gradient.backward()
            self.actor_optimizer.step()

            # target networks
            self.update_targets()

        self.update_step += 1

        torch.save(self.critic1.state_dict(), "./critic1.pt")
        torch.save(self.critic2.state_dict(), "./critic2.pt")
        torch.save(self.critic1_target.state_dict(), "./critic1_target.pt")
        torch.save(self.critic2_target.state_dict(), "./critic2_target.pt")
        torch.save(self.actor.state_dict(), "./actor.pt")
        torch.save(self.actor_target.state_dict(), "./actor_target.pt")   

    def generate_action_space_noise(self, action_batch):
        noise = torch.normal(torch.zeros(action_batch.size()), self.noise_std).clamp(-self.noise_bound, self.noise_bound).to(self.device)
        return noise

    def update_targets(self):
        for target_param, param in zip(self.critic1_target.parameters(), self.critic1.parameters()):
            target_param.data.copy_(param.data * self.tau + target_param.data * (1.0 - self.tau))

        for target_param, param in zip(self.critic2_target.parameters(), self.critic2.parameters()):
            target_param.data.copy_(param.data * self.tau + target_param.data * (1.0 - self.tau))
        
        for target_param, param in zip(self.actor_target.parameters(), self.actor.parameters()):
            target_param.data.copy_(param.data * self.tau + target_param.data * (1.0 - self.tau))