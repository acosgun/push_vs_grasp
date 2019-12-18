import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.autograd as autograd
import numpy as np
import cv2


class Critic(nn.Module):

    def __init__(self, obs_dims, action_dim):
        super(Critic, self).__init__()

        self.obs_dim = obs_dims
        self.action_dim = action_dim


        self.conv1 = nn.Conv2d(3, 1, 5)
        self.flatten = Flatten()

        self.linear1 = nn.Linear(5376, 1024)
        self.linear2 = nn.Linear(1024 + self.action_dim, 512)
        self.linear3 = nn.Linear(512, 300)
        self.linear4 = nn.Linear(300, 1)

    def forward(self, x, a):
      
        x = F.leaky_relu(self.conv1(x))
        x = self.flatten(x)
        x = F.leaky_relu(self.linear1(x))
        xa_cat = torch.cat([x,a], 1)
        xa = F.leaky_relu(self.linear2(xa_cat))
        xa = F.leaky_relu(self.linear3(xa))
        qval = F.sigmoid(self.linear4(xa))

        return qval

class Flatten(nn.Module):
    def __init__(self):
        super(Flatten, self).__init__()

    def forward(self, x):
        return x.view(x.size(0), -1)

class Actor(nn.Module):

    def __init__(self, obs_dim, action_dim):
        super(Actor, self).__init__()

        self.obs_dim = obs_dim
        self.action_dim = action_dim

        self.conv1 = nn.Conv2d(3, 1, 10)
        self.flatten = Flatten()
        self.linear2 = nn.Linear(4641, 128)

        self.linear3 = nn.Linear(128, self.action_dim)

    def forward(self, obs): 
        
        
        x = F.leaky_relu(self.conv1(obs))
        x = self.flatten(x)
        x = F.leaky_relu(self.linear2(x))
        x = F.sigmoid(self.linear3(x))

        return x