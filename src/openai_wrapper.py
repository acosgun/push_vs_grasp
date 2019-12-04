import gym
from gym import spaces
import math
import rospy
from push_v_grasp_ws.msg import push_action


HEIGHT = 500
WIDTH = 500
N_CHANNELS = 3

class CustomEnv(gym.Env):
  """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, arg1, arg2, ...):
        super(CustomEnv, self).__init__()    

        self.action_space = spaces.Box(low=np.array([0, 0, 0, 0]), high=np.array([50, 50, 2*math.pi, 50]), dtype=np.float16))       
        
        #Image as Input using
        self.observation_space = spaces.Box(low=0, high=255, shape=(HEIGHT, WIDTH, N_CHANNELS), dtype=np.uint8)

        self.reset_service = rospy.ServiceProxy('push_action', push_action)


    def step(self, action):
        pass

    # Execute one time step within the environment
    def reset(self):
        pass

    # Reset the state of the environment to an initial state
    def render(self, mode='human', close=False):
        pass
    