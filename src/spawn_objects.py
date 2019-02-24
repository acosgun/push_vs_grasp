#!/usr/bin/env python

import os, rospy, tf
from math import sqrt
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelState, LinkState
import actionlib
import push_vs_grasp.msg

max_num_objs = 10
object_radius = 0.03

class GenerateCylinders(object):    
    def __init__(self, name):        
        self.model_filename_obj_1 = rospy.get_param('~model_filename_obj_1')
        self.model_filename_obj_2 = rospy.get_param('~model_filename_obj_2')        
        self.model_filename_obj_3 = rospy.get_param('~model_filename_obj_3')
        self.model_filename_obj_4 = rospy.get_param('~model_filename_obj_4')        
        
        self.model_name = "unit_cylinder"
        self.model_name_goal = "unit_cylinder_goal"
        
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        rospy.wait_for_service("gazebo/pause_physics")
        rospy.wait_for_service("gazebo/unpause_physics")
        rospy.wait_for_service("gazebo/get_model_state")
        
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        
        fout_obj_1 = open(self.model_filename_obj_1, "r")
        fout_obj_2 = open(self.model_filename_obj_2, "r")
        fout_obj_3 = open(self.model_filename_obj_3, "r")
        fout_obj_4 = open(self.model_filename_obj_4, "r")
        self.obj_xml_1 = fout_obj_1.read()
        self.obj_xml_2 = fout_obj_2.read()
        self.obj_xml_3 = fout_obj_3.read()
        self.obj_xml_4 = fout_obj_4.read() 
        fout_obj_1.close()
        fout_obj_2.close()
        fout_obj_3.close()
        fout_obj_4.close()
                
        self._as = actionlib.SimpleActionServer("generate_cylinders", push_vs_grasp.msg.GenerateCylindersAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        print "GenerateCylinders Server ON"

    def get_all_objects(self):
        centroids = []
        for i in range(max_num_objs):
            cur_obj_name = self.model_name + str(i)
            response = self.get_model_state(cur_obj_name,  "robot::base_link")
            if response.success:
                centroid = geometry_msgs.msg.PointStamped()        
                centroid.point = response.pose.position
                centroids.append(centroid)
            else:
                break
        return centroids

        
    def execute_cb(self, goal):
        print "GenerateCylinders execute_cb"

        if goal.get_perfect_perception:
            result = push_vs_grasp.msg.GenerateCylindersResult()
            result.centroids = self.get_all_objects()
            self._as.set_succeeded(result)
            return
                        
        cur_model_name = self.model_name_goal + str(0)
        self.delete_model(cur_model_name)
        blue_goal_pose = Pose(goal.blue_goal.point, Quaternion(0,0,0,0))
        cur_xml = self.obj_xml_3
        self.spawn_sdf_model(cur_model_name, cur_xml, "", blue_goal_pose, "robot::base_link")        
        cur_model_name = self.model_name_goal + str(1)
        self.delete_model(cur_model_name)
        red_goal_pose = Pose(goal.red_goal.point, Quaternion(0,0,0,0))
        cur_xml = self.obj_xml_4
        self.spawn_sdf_model(cur_model_name, cur_xml, "", red_goal_pose, "robot::base_link")

        
        result = push_vs_grasp.msg.GenerateCylindersResult()        
        num_objs = goal.num_objs
        min_x = goal.min_x
        max_x = goal.max_x
        min_y = goal.min_y
        max_y = goal.max_y

        if num_objs % 2 == 0:
            num_blue = int(num_objs/2)
        else:
            import math
            import random
            if  random.random() > 0.5:
                num_blue = round(math.ceil(float(num_objs)/2))
            else:
                num_blue = round(math.floor(float(num_objs)/2))
                
        print "num_objs: " + str(num_objs)
        print "num_blue: " + str(num_blue)

        self.pause_physics()
        while len(result.centroids) < num_objs:
            import random
            x = random.uniform(min_x,max_x)
            y = random.uniform(min_y,max_y)
            z = 0.05
        
            #check collisions with others in the list
            coll_radius = 0.08
            for j in result.centroids:
                if sqrt((j.point.x-x)**2 + (j.point.y-y)**2) < coll_radius:
                    continue
                                            
            if len(result.centroids) < num_blue:                                
                cur_xml = self.obj_xml_1
                if sqrt((goal.blue_goal.point.x - x)**2 +(goal.blue_goal.point.y - y)**2) < (goal.blue_radius + object_radius):
                    continue
                result.colors.append("blue")
            else:
                cur_xml = self.obj_xml_2
                if sqrt((goal.red_goal.point.x - x)**2 + (goal.red_goal.point.y - y)**2) < (goal.red_radius + object_radius):
                    continue
                result.colors.append("red")
                    
            cur_model_name = self.model_name + str(len(result.centroids))


            self.delete_model(cur_model_name)
            item_pose = Pose(Point(x,y,z), Quaternion(0,0,0,0))
            self.spawn_sdf_model(cur_model_name, cur_xml, "", item_pose, "robot::base_link")
            
            #add object to list
            pt = geometry_msgs.msg.PointStamped()
            pt.point.x = x
            pt.point.y = y
            pt.point.z = z
            result.centroids.append(pt)
            result.radiuses.append(object_radius)

        self.unpause_physics()
        self._as.set_succeeded(result)
                
if __name__ == '__main__':
    rospy.init_node('generate_cylinders')
    server = GenerateCylinders(rospy.get_name())
    rospy.spin()
