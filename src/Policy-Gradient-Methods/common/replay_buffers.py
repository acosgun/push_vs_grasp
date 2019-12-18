import random
import numpy as np
from collections import deque
import torch

class BasicBuffer:

    def __init__(self, max_size):
        self.max_size = max_size
        self.buffer = deque(maxlen=max_size)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


    def push(self, state, action, reward, next_state, done):

        unsqueeze = lambda x : torch.unsqueeze(x,0)

        action = unsqueeze(torch.FloatTensor(action).to(self.device))

        reward = unsqueeze(torch.FloatTensor(np.array([reward])).to(self.device))
        done = unsqueeze(torch.FloatTensor([done]).to(self.device))

        experience = (state, action, reward, next_state, done)

        #experience = (state, action, np.array([reward]), next_state, done)
        self.buffer.append(experience)

    def sample(self, batch_size):
        print("length of buffer is: " + str(len(self)))
        state_batch = torch.FloatTensor([]).to(self.device)
        action_batch = torch.FloatTensor([]).to(self.device)
        reward_batch = torch.FloatTensor([]).to(self.device)
        next_state_batch = torch.FloatTensor([]).to(self.device)
        done_batch = torch.Tensor([]).to(self.device)

        batch = random.sample(self.buffer, batch_size)

        for experience in batch:

            state, action, reward, next_state, done = experience
            state_batch = torch.cat((state_batch, state),0)
            action_batch = torch.cat((action_batch, action),0)
            reward_batch = torch.cat((reward_batch, reward),0)
            next_state_batch = torch.cat((next_state_batch, next_state),0)
            done_batch = torch.cat((done_batch, done),0)

            # state_batch.append(state)
            # action_batch.append(action)
            # reward_batch.append(reward)
            # next_state_batch.append(next_state)
            # done_batch.append(done)



        return (state_batch, action_batch, reward_batch, next_state_batch, done_batch)
      #  return (torch.FloatTensor(state_batch).to(self.device), torch.FloatTensor(action_batch).to(self.device), torch.FloatTensor(reward_batch).to(self.device), torch.FloatTensor(next_state_batch).to(self.device), torch.FloatTensor(done_batch).to(self.device))

    def sample_sequence(self, batch_size):
        state_batch = []
        action_batch = []
        reward_batch = []
        next_state_batch = []
        done_batch = []

        min_start = len(self.buffer) - batch_size
        start = np.random.randint(0, min_start)

        for sample in range(start, start + batch_size):
            state, action, reward, next_state, done = self.buffer[start]
            state, action, reward, next_state, done = experience
            state_batch.append(state)
            action_batch.append(action)
            reward_batch.append(reward)
            next_state_batch.append(next_state)
            done_batch.append(done)

        return (state_batch, action_batch, reward_batch, next_state_batch, done_batch)

    def __len__(self):
        return len(self.buffer)
