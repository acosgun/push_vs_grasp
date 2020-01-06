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
import torch
import random
import subprocess, time



HEIGHT = 500
WIDTH = 900
N_CHANNELS = 3

class CustomEnv(gym.Env):
#   """Custom Environment that follows gym interface"""
    #metadata = {'render.modes': ['human']}

    def __init__(self):
        super(CustomEnv, self).__init__() 
   
        self.bridge = CvBridge()
        self.action_space = spaces.Box(low=np.array([-37.5, 5, 0, 30]), high=np.array([37.5, 35, 2*math.pi, 50]), dtype=np.float16)       

        #Image as Input using
        self.observation_space = spaces.Box(low=0, high=255, shape=(HEIGHT, WIDTH, N_CHANNELS), dtype=np.uint8)

        self.restart_simulator()

    def step(self, action):
        #input is a numpy array containing the four parameters for the action
        start_x = 75 * (action[0] - 0.5)
        start_y = action[1] * 30 + 5
        end_x = 75 * (action[2] - 0.5)
        end_y = action[3] * 30 + 5

        print(start_x, start_y, end_x, end_y)
        try:
            response = self.push_service(start_x, start_y, end_x, end_y)

            img = response.next_state

            l = []
            for i in response.objects:
                l.append(i.x)
                l.append(i.y)
                l.append(int(i.is_red))    
            l = torch.Tensor(l).long().cuda()    


            cv_image = self.bridge.imgmsg_to_cv2(img, "rgb8")
            cv2.imwrite("/home/rhys/pic.png", cv_image)
            # image = np.expand_dims(cv_image, axis=0)
            # image = torch.tensor(np.transpose(image, (0,3,1,2))).to(torch.device("cuda"), dtype=torch.float)

            return [l, response.reward, response.done, -1]


        except:
            self.restart_simulator()
            return self.step(action)
            

       
    def reset(self):
        try:
        
            response = self.reset_service()
            # img = response.next_state
            # cv_image = self.bridge.imgmsg_to_cv2(img, "rgb8")
            # image = np.expand_dims(cv_image, axis=0)
            # image = torch.tensor(np.transpose(image, (0,3,1,2))).to(torch.device("cuda"), dtype=torch.float)
            # return image # img = response.next_state
            # cv_image = self.bridge.imgmsg_to_cv2(img, "rgb8")
            # image = np.expand_dims(cv_image, axis=0)
            # image = torch.tensor(np.transpose(image, (0,3,1,2))).to(torch.device("cuda"), dtype=torch.float)
            # return image
            l = []
            for i in response.objects:
                l.append(i.x)
                l.append(i.y)
                l.append(int(i.is_red))    
            l = torch.Tensor(l).long().cuda()
            return l
        except:
            self.restart_simulator()
            return self.reset()


        
    def restart_simulator(self):
        subprocess.Popen(["/home/rhys/gym_test/ws/src/push_vs_grasp/src/simulator.sh"], shell=True)
        
        rospy.wait_for_service('/reset_action')
        rospy.wait_for_service('/push_action')

        self.reset_service = rospy.ServiceProxy('/reset_action', reset)

        self.push_service = rospy.ServiceProxy('/push_action', push_action)


    # Reset the state of the environment to an initial state
    def render(self, mode='human', close=False):
        pass

# CustomEnv()