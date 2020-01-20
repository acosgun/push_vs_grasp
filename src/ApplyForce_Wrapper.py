#! /usr/bin/env python

import rospy, actionlib
import random
from geometry_msgs.msg import PointStamped
from push_vs_grasp.msg import PushAction, PushGoal, PushResult
import numpy as np
from gym import spaces

class wrapper:

    def generate_objects(self):

        coll_radius = 0.02

        min_x = -0.5
        max_x = -0.2

        min_y = 0
        max_y = 0.3

        objects = []
        radiuses = []
        colours = []

        while len(objects) < 5:
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)

            collision_check = map(lambda p : (p.point.x - x) * (p.point.x - x) + (p.point.y - y) * (p.point.y - y) < coll_radius * coll_radius, objects)

            if any(collision_check):
                continue
            else:
                p = PointStamped()
                p.point.x = x  
                p.point.y = y
                objects.append(p)

                colours.append("blue") # if random.random() > 0.5 else "rec")

                radiuses.append(coll_radius)

        r1 = PointStamped()
        r1.point.x = -0.7 + 0.2
        r1.point.y = -(0.75 - 0.2)

        r2 = PointStamped()
        r2.point.x = -0.1
        r2.point.y = (0.75 - 0.2)

        goal_radiuses = []
        goal_radiuses.append(0.2)
        goal_radiuses.append(0.2)

        


        return [objects, radiuses, colours, r1, r2, goal_radiuses]

    def reset(self):
        self.objects, self.radiuses, self.colours, self.r1, self.r2, self.goal_radiuses = self.generate_objects()
        return self.get_object_list()


    def step(self, action):
        
        start_x, start_y, end_x, end_y = action
        goal = PushGoal()

        goal.centroids = self.objects
        goal.radiuses = self.radiuses
        goal.colors = self.colours
        goal.goal_radiuses = self.goal_radiuses
        goal.red_goal = self.r1
        goal.blue_goal = self.r2

        # Top Left Corner = -35, 25
        # Bottom Left = -35, 0
        # Top Right = 35, 25
        
        # -35 < start_x < 35
        # 0 < end_y < 30

        # -30 < end_x  < 30
        # 5 < end_y < 25

        goal.start_x = (start_x - 0.5) * 70
        goal.end_x = (end_x - 0.5) * 60

        goal.start_y = start_y * 30
        goal.end_y = (end_y * 20) + 5

        self.client.send_goal(goal)

        self.client.wait_for_result()
        self.objects = []
        self.colours = []

        result = self.client.get_result()

        for i in result.objects:

            p = PointStamped()
            p.point.x = i.x  
            p.point.y = i.y
            self.objects.append(p)

            self.colours.append("red" if i.is_red else "blue")

        if result.done:
            input()
        return [self.get_object_list(), result.reward, result.done, -1]

    def get_object_list(self):

        return sum([[int((a+1)*50),int((b+1)*50),int(c)+1] for ((a,b),c) in zip(map(lambda x : (x.point.x, x.point.y) , self.objects), map(lambda x : 0 if x == "red" else 1, self.colours))], [])

    

    def __init__(self):

        rospy.init_node('pick_place_server')

        self.objects, self.radiuses, self.colours, self.r1, self.r2, self.goal_radiuses = self.generate_objects()

        self.client = actionlib.SimpleActionClient('box2d_planner', PushAction)

        self.action_space = spaces.Box(np.array([0,0,0,0]), np.array([1,1,1,1]))

        self.observation_space = spaces.Box(np.zeros(15), np.ones(15) * 100)

        self.client.wait_for_server()

        # self.step(0.2,0.2,0,0)

        # print(self.get_object_list())