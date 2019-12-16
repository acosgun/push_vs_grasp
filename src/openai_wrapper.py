#!/usr/bin/env python
import gym
import numpy as np
from gym import spaces
import math
import rospy
from push_vs_grasp.srv import push_action
from push_vs_grasp.srv import reset
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



HEIGHT = 500
WIDTH = 900
N_CHANNELS = 3

class CustomEnv(gym.Env):
#   """Custom Environment that follows gym interface"""
    #metadata = {'render.modes': ['human']}

    def __init__(self):
        super(CustomEnv, self).__init__() 
   
        self.bridge = CvBridge()
        self.action_space = spaces.Box(low=np.array([0, 0, 0, 0]), high=np.array([50, 50, 2*math.pi, 50]), dtype=np.float16)       

        #Image as Input using
        self.observation_space = spaces.Box(low=0, high=255, shape=(HEIGHT, WIDTH, N_CHANNELS), dtype=np.uint8)

        rospy.wait_for_service('/reset_action')
        rospy.wait_for_service('/push_action')

        self.reset_service = rospy.ServiceProxy('/reset_action', reset)

        self.push_service = rospy.ServiceProxy('/push_action', push_action)

        
        x = np.array([10,20,1.57,30])
        print(x)
        while True:
            self.step(x)
            #self.reset()
            raw_input()

            

    def step(self, action):
        #input is a numpy array containing the four parameters for the action
        print(action)
        start_x = action[0]
        start_y = action[1]
        angle = action[2]
        push_dist = action[3]

        response = self.push_service(start_x, start_y, angle, push_dist)

        img = response.next_state

        cv_image = self.bridge.imgmsg_to_cv2(img, "rgb8")
        cv2.imwrite("/home/rhys/pic.png", cv_image)

    def reset(self):
        print("about to reset")
        self.reset_service()


    # Reset the state of the environment to an initial state
    def render(self, mode='human', close=False):
        pass

a = CustomEnv()