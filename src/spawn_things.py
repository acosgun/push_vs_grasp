#!/usr/bin/env python

import os, rospy, tf
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Point, Quaternion

if __name__ == '__main__':
    
    rospy.init_node("spawn_table_node")

    model_filename = rospy.get_param('~model_filename')
    model_name = rospy.get_param('~model_name')
    world_frame = rospy.get_param('~world_frame')
    x = rospy.get_param('~x')
    y = rospy.get_param('~y')
    z = rospy.get_param('~z')
    roll = rospy.get_param('~roll')
    pitch = rospy.get_param('~pitch')
    yaw = rospy.get_param('~yaw')

    
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)    
    
    with open(model_filename, "r") as f:        
        product_xml = f.read()

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        
        spawn_sdf_model(model_name, product_xml, "", pose, world_frame)
