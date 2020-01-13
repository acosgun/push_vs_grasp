import numpy as np, cv2
import math
import gym
import torch

class mini_batch_train:
    def start(self, env, agent, max_episodes, max_steps, batch_size):
        self.episode_rewards = []
        episode = 0
        while True:
            episode += 1
        #for episode in range(max_episodes):
            state = env.reset().to('cuda').long()
            print("EPISODE..." + str(episode))
            episode_reward = 0

            for step in range(max_steps):
                action = agent.get_action(torch.unsqueeze(state,dim=0))
               
                next_state, reward, done, _ = env.step(action)
                # print(next_state)
                agent.replay_buffer.push(state, action, reward, next_state, done, reward > 0.01)
                episode_reward += reward

                if len(agent.replay_buffer) > batch_size:
                    agent.update(batch_size)   

                if done or step == max_steps-1:
                    self.episode_rewards.append(episode_reward)
                    print("Episode " + str(episode) + ": " + str(episode_reward))
                    break
                
                state = next_state.to('cuda').long()
                

        #return self.episode_rewards

def mini_batch_train_frames(env, agent, max_frames, batch_size):
    episode_rewards = []
    state = env.reset()
    episode_reward = 0

    for frame in range(max_frames):
        action = agent.get_action(state)
        next_state, reward, done, _ = env.step(action)
        agent.replay_buffer.push(state, action, reward, next_state, done)
        episode_reward += reward

        if len(agent.replay_buffer) > batch_size:
            agent.update(batch_size)   

        if done:
            episode_rewards.append(episode_reward)
            print("Frame " + str(frame) + ": " + str(episode_reward))
            state = env.reset()
            episode_reward = 0
        
        state = next_state
            
    return episode_rewards

# process episode rewards for multiple trials
def process_episode_rewards(many_episode_rewards):
    minimum = [np.min(episode_reward) for episode_reward in episode_rewards]
    maximum = [np.max(episode_reward) for episode_reward in episode_rewards]
    mean = [np.mean(episode_reward) for episode_reward in episode_rewards]

    return minimum, maximum, mean