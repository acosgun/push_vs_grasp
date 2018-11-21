#!/usr/bin/env python

import os, rospy, tf
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Point, Quaternion

if __name__ == '__main__':

    rospy.init_node("spawn_table_node")

    table_model_filename = rospy.get_param('~model_filename')
    
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    
    world_frame = "world"            
    
    with open(table_model_filename, "r") as f:        
        product_xml = f.read()
        item_pose = Pose(Point(0.0,0.0,0.0), Quaternion(0,0,0,0))
        spawn_sdf_model("table", product_xml, "", item_pose, world_frame)
