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

        self.current_object_state = None    
        #Image as Input using
        self.observation_space = spaces.Box(low=0, high=255, shape=(HEIGHT, WIDTH, N_CHANNELS), dtype=np.uint8)

        self.restart_simulator()

    def step(self, action, retain=False):
        #input is a numpy array containing the four parameters for the action
        print(action)
        start_x = 75 * (action[0] - 0.5)
        start_y = action[1] * 30 + 5
        end_x = 75 * (action[2] - 0.5)
        end_y = action[3] * 30 + 5

        

        print(start_x, start_y, end_x, end_y)
        raw_input()
        try:
            
            response = self.push_service(start_x, start_y, end_x, end_y, self.current_object_state)

            #img = response.next_state

            l = []
            #print(response.objects)
            for i in self.current_object_state:
                x,y = self.box_2d_to_img(i.x, i.y)
                l.append(x)
                l.append(y)
                l.append(int(i.is_red))    
            l = torch.Tensor(l).long().cuda() 

            # if retain:
            #     self.current_object_state = response.objects   


            # cv_image = self.bridge.imgmsg_to_cv2(img, "rgb8")
            # cv2.imwrite("/home/rhys/pic.png", cv_image)
        # image = np.expand_dims(cv_image, axis=0)
        # image = torch.tensor(np.transpose(image, (0,3,1,2))).to(torch.device("cuda"), dtype=torch.float)

            # return [l, response.reward, response.done, -1]
            return [l, 0,0,-1]

        except Exception as e:
            print(e)
            self.restart_simulator()
            return self.step(action)
            

    def box_2d_to_img(self, x_in, y_in):
        return (x_in + 37.5) + 10, -(y_in + 10) + 50
       
    def reset(self):
        try:
        
            response = self.reset_service()

            l = []
            for i in response.objects:
                x,y = self.box_2d_to_img(i.x, i.y)
                l.append(x)
                l.append(y)
                l.append(int(i.is_red))    

            l = torch.Tensor(l).long().cuda()
            self.current_object_state = response.objects   
            return l
        except Exception as e:
            print(e)

            self.restart_simulator()
            return self.reset()


        
    def restart_simulator(self):
        subprocess.Popen(["/home/rhys/catkin_ws/src/push_vs_grasp/src/simulator.sh"], shell=True)
        
        rospy.wait_for_service('/reset_action')
        rospy.wait_for_service('/push_action')

        self.reset_service = rospy.ServiceProxy('/reset_action', reset)

        self.push_service = rospy.ServiceProxy('/push_action', push_action)


    # Reset the state of the environment to an initial state
    def render(self, mode='human', close=False):
        pass

# CustomEnv()
