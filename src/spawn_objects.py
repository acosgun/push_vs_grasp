#!/usr/bin/env python

import os, rospy, tf
from math import sqrt
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelState, LinkState

if __name__ == '__main__':

    rospy.init_node("spawn_objects")
    model_filename_obj_1 = rospy.get_param('~model_filename_obj_1')
    model_filename_obj_2 = rospy.get_param('~model_filename_obj_2')
        
    world_frame = "world"

    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    #rospy.wait_for_service("gazebo/spawn_urdf_model")
    #rospy.wait_for_service("gazebo/set_model_state")
    #rospy.wait_for_service("gazebo/get_model_state")
    #rospy.wait_for_service("gazebo/get_link_state")
    #rospy.wait_for_service("gazebo/set_link_state")
    #rospy.wait_for_service("gazebo/reset_world")
    rospy.wait_for_service("gazebo/pause_physics")
    rospy.wait_for_service("gazebo/unpause_physics")
    
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    #spawn_urdf_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    #set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    #get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    #get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    #set_link_state = rospy.ServiceProxy("/gazebo/set_link_state", SetLinkState)
    #reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
    unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        
    num_objs = 5
    model_name = "unit_cylinder"
    obj_list = []

    from random import randint
    num_first_color = randint(0,num_objs)

    min_x = -0.4
    max_x = 0.4
    min_y = -0.35
    max_y = 0.05
        
    fout_obj_1 = open(model_filename_obj_1, "r")
    fout_obj_2 = open(model_filename_obj_2, "r")
    obj_xml_1 = fout_obj_1.read()
    obj_xml_2 = fout_obj_2.read() 
    fout_obj_1.close()
    fout_obj_2.close()
            
    pause_physics()
        
    while len(obj_list) < num_objs:
        import random
        x = random.uniform(min_x,max_x)
        y = random.uniform(min_y,max_y)
        z = 1.08
            
        #check collisions with others in the list
        collision = False
        coll_radius = 0.08
        for j in obj_list:
            if sqrt((j.x-x)**2 + (j.y-y)**2) < coll_radius:
                collision = True
                break
            
        if collision == False:                
            
            #Spawn Object
            item_pose = Pose(Point(x,y,z), Quaternion(0,0,0,0))

            if len(obj_list) < num_first_color:
                cur_xml = obj_xml_1
            else:
                cur_xml = obj_xml_2

            cur_model_name = model_name + str(len(obj_list))
            delete_model(cur_model_name)
            
            spawn_sdf_model(cur_model_name, cur_xml, "", item_pose, "table::link")
            
            #add object to list
            pt = geometry_msgs.msg.Point()
            pt.x = x
            pt.y = y
            pt.z = z
            obj_list.append(pt)
    unpause_physics()    
