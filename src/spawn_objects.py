#!/usr/bin/env python

import os, rospy, tf
from math import sqrt
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelState, LinkState
import actionlib
import push_vs_grasp.msg

class GenerateCylinders(object):

    
    def __init__(self, name):        
        self.model_filename_obj_1 = rospy.get_param('~model_filename_obj_1')
        self.model_filename_obj_2 = rospy.get_param('~model_filename_obj_2')        

        self.model_name = "unit_cylinder"
        
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        rospy.wait_for_service("gazebo/pause_physics")
        rospy.wait_for_service("gazebo/unpause_physics")

        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)

        fout_obj_1 = open(self.model_filename_obj_1, "r")
        fout_obj_2 = open(self.model_filename_obj_2, "r")
        self.obj_xml_1 = fout_obj_1.read()
        self.obj_xml_2 = fout_obj_2.read() 
        fout_obj_1.close()
        fout_obj_2.close()
                
        self._as = actionlib.SimpleActionServer("generate_cylinders", push_vs_grasp.msg.GenerateCylindersAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        print "GenerateCylinders Server ON"
        
    def execute_cb(self, goal):
        print "GenerateCylinders execute_cb"

        result = push_vs_grasp.msg.GenerateCylindersResult()
        
        num_objs = goal.num_objs
        min_x = goal.min_x
        max_x = goal.max_x
        min_y = goal.min_y
        max_y = goal.max_y


        self.pause_physics()
    
        while len(result.centroids) < num_objs:
            import random
            x = random.uniform(min_x,max_x)
            y = random.uniform(min_y,max_y)
            z = 0.05
        
            #check collisions with others in the list
            collision = False
            coll_radius = 0.08
            for j in result.centroids:
                if sqrt((j.point.x-x)**2 + (j.point.y-y)**2) < coll_radius:
                    collision = True
                    break
            
            if collision == False:                
            
                #Spawn Object
                item_pose = Pose(Point(x,y,z), Quaternion(0,0,0,0))
                
                import random
                if random.random() > 0.5:
                    cur_xml = self.obj_xml_1
                    result.colors.append("blue")
                else:
                    cur_xml = self.obj_xml_2
                    result.colors.append("red")

                cur_model_name = self.model_name + str(len(result.centroids))
                self.delete_model(cur_model_name)
                
                #self.spawn_sdf_model(cur_model_name, cur_xml, "", item_pose, "table::link")
                self.spawn_sdf_model(cur_model_name, cur_xml, "", item_pose, "robot::base_link")
                
                #add object to list
                pt = geometry_msgs.msg.PointStamped()
                pt.point.x = x
                pt.point.y = y
                pt.point.z = z
                result.centroids.append(pt)
                self.unpause_physics()    
                
        self._as.set_succeeded(result)
                
if __name__ == '__main__':
    rospy.init_node('generate_cylinders')
    server = GenerateCylinders(rospy.get_name())
    rospy.spin()
