import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.autograd as autograd
import numpy as np
import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
from torch import FloatTensor
from torch.autograd import Variable



class InvariantModel(nn.Module):
    def __init__(self, phi, rho):
        super(InvariantModel, self).__init__()
        self.phi = phi
        self.rho = rho

    def forward(self, x, a = None):
        # compute the representation for each data point
        # print(x)
        # print(x.shape)
        # print(x)
        # print("original shape: " + str(x.shape))
        x = self.phi.forward(x)
        # print(x)
        # print("shape after network: " + str(x.shape))
        # print(x)
        x = torch.sum(x, dim=1, keepdim=False)
        # print("shape after summing: " + str(x.shape))
        # print(x)

        if not (a is None):
            # print(a)
            # print(a.shape)
            # print(x)
            # print(x.shape)
            
            # raw_input()
            # print(x)
            # print(a)
            x = torch.cat([x.float(),a.float().cuda()], 1)
     
        out = self.rho.forward(x)
        # raw_input()
        # print("output of rho")
        # print(out)
        # print(self.rho.output_size)
        return out

class TextSumer(nn.Module):

    def __init__(self):
        super(TextSumer, self).__init__()

        self.embedding = nn.Embedding(500, 100)
        self.linear = nn.Linear(100, 30)
        self.tanh = nn.Tanh()

    def forward(self, x):
        x = self.embedding(x)
        x = self.linear(x)
        x = self.tanh(x)
        return x

class SmallRho(nn.Module):
    def __init__(self, input_size, output_size = 1):
        super(SmallRho, self).__init__()
        self.input_size = input_size
        self.output_size = output_size

        self.fc1 = nn.Linear(self.input_size, 10)
        self.fc2 = nn.Linear(10, self.output_size)

    def forward(self, x):
        # print("next network: " + str(x.shape))
        x = F.relu(self.fc1(x))
        # print("Going through first layer: " + str(x.shape))
        x = torch.sigmoid(self.fc2(x)).squeeze(dim=0).cpu() #.detach().numpy()
        # print("final shapee: " + str(x.shape))
       # x = self.fc2(x)
        return x


class Critic(nn.Module):

    def __init__(self, obs_dims, action_dim):
        super(Critic, self).__init__()

        self.obs_dim = obs_dims
        self.action_dim = action_dim

        self.the_phi = TextSumer()
        self.the_rho = SmallRho(input_size=34, output_size=1)

        self.model = InvariantModel(phi=self.the_phi, rho=self.the_rho)

    def forward(self, x, a):

        return self.model.forward(x,a)
   

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

        self.the_phi = TextSumer()
        self.the_rho = SmallRho(input_size=30, output_size=self.action_dim)

        self.model = InvariantModel(phi=self.the_phi, rho=self.the_rho)

    def forward(self, obs): 
        
        return self.model.forward(obs)
