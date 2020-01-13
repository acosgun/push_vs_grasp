from td3 import TD3Agent
from common.utils import mini_batch_train
import gym,sys

sys.path.append('/home/rhys/catkin_ws/src/push_vs_grasp/src')
import openai_wrapper

# env = gym.make("Pendulum-v0")
env = openai_wrapper.CustomEnv()

gamma = 0.99
tau = 1e-2
noise_std = 0.2
bound = 0.5
delay_step = 2
buffer_maxlen = 500
critic_lr = 1e-5
actor_lr = 1e-5

max_episodes = 100000
max_steps = 200
batch_size = 10
agent = TD3Agent(env, gamma, tau, buffer_maxlen, delay_step, noise_std, bound, critic_lr, actor_lr)

train = mini_batch_train()
try:
    # def mini_batch_train(env, agent, max_episodes, max_steps, batch_size):
    train.start(env, agent, max_episodes, max_steps, batch_size)
except KeyboardInterrupt:
    print(train.episode_rewards)

