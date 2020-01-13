import random
import numpy as np
from collections import deque
import torch
import glob

class BasicBuffer:

    def __init__(self, max_size):
        self.max_size = max_size
        self.buffer = deque(maxlen=max_size * 2/3)
        self.no_reward_buffer = deque(maxlen=max_size / 3)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.number = 0

        load_previous = False

        if not load_previous:
            return

        for i in glob.glob("./samples/*.pt"):
            file_name = i.split("/")[-1].split(".")[0]
            print(file_name)
            
            self.number = max(self.number, int(file_name.split("_")[0]))
            reward_change = file_name.split("_")[1] == "c"

            experience = torch.load(i)

            if reward_change:
                self.buffer.append(experience)
            else:
                self.no_reward_buffer.append(experience)




            


    def push(self, state, action, reward, next_state, done, reward_change):

        self.number += 1

        
        unsqueeze = lambda x : torch.unsqueeze(x,0)
    
        action = unsqueeze(torch.FloatTensor(action).to(self.device))

        reward = unsqueeze(torch.FloatTensor(np.array([reward])).to(self.device))
        done = unsqueeze(torch.LongTensor([done]).to(self.device))

        experience = (state, action, reward, next_state, done)

        #experience = (state, action, np.array([reward]), next_state, done)
        if reward_change:
            self.buffer.append(experience)
            torch.save(experience, "./samples/" + str(self.number) + "_c.pt")

        else:
            self.no_reward_buffer.append(experience)
            torch.save(experience, "./samples/" +  str(self.number) + "_nc.pt")


    def sample(self, batch_size):
        print("length of buffer is: " + str(len(self)))
        state_batch = torch.LongTensor([]).to(self.device)
        action_batch = torch.FloatTensor([]).to(self.device)
        reward_batch = torch.FloatTensor([]).to(self.device)
        next_state_batch = torch.LongTensor([]).to(self.device)
        
        done_batch = torch.Tensor([]).to(self.device)

        batch = random.sample(self.buffer, int(batch_size * 2/3.0)) + random.sample(self.no_reward_buffer, int(batch_size / 3.0))

        for experience in batch:

            state, action, reward, next_state, done = experience
            state_batch = torch.cat((state_batch, torch.unsqueeze(state,dim=0)),0)
            action_batch = torch.cat((action_batch, action),0)
            reward_batch = torch.cat((reward_batch, reward),0)
            next_state_batch = torch.cat((next_state_batch, torch.unsqueeze(next_state,dim=0)),0)
            done_batch = torch.cat((done_batch, done.float()),0)
        # print(batch)
        # print(len(self.buffer))
        # print(state_batch)
        # print(action_batch)
        # print(state_batch.shape)
        # print(action_batch.shape)
        #raw_input()

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
        return int(min(len(self.buffer), len(self.no_reward_buffer)))
