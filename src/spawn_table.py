#!/usr/bin/env python

import os, rospy, tf
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelState, LinkState

if __name__ == '__main__':

    world_frame = "world"

    print("Waiting for gazebo services...")
    rospy.init_node("model_spawner")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    #rospy.wait_for_service("gazebo/spawn_urdf_model")
    #rospy.wait_for_service("gazebo/set_model_state")
    #rospy.wait_for_service("gazebo/get_model_state")
    #rospy.wait_for_service("gazebo/get_link_state")
    #rospy.wait_for_service("gazebo/set_link_state")
    #rospy.wait_for_service("gazebo/reset_world")
    #rospy.wait_for_service("gazebo/pause_physics")
    print("Gazebo services OK.")    
    
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    #spawn_urdf_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    #set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    #get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    #get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    #set_link_state = rospy.ServiceProxy("/gazebo/set_link_state", SetLinkState)
    #reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    #pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
    #unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        
    with open("/home/acos0001/catkin_ws/src/push_vs_grasp/models/sdf/table/model.sdf", "r") as f:        
        product_xml = f.read()
        item_pose = Pose(Point(0.0,0.0,0.0), Quaternion(0,0,0,0))
        delete_model("table")
        spawn_sdf_model("table", product_xml, "", item_pose, world_frame)

    #set_link_state(LinkState("robot::base_link", Pose(Point(0,1.0,1.0), Quaternion(0,0,0,0)), Twist(Vector3(0,0,0), Vector3(0,0,0)), "world"))
    #with open("/home/acos0001/catkin_ws/src/push_vs_grasp/urdf/ur5_joint_limited_robot.urdf.xacro", "r") as f:
    #file_location = "/home/acos0001/catkin_ws/src/push_vs_grasp/urdf/ur5_joint_limited_robot.urdf.xacro"
    #p = os.popen("rosrun xacro xacro.py " + file_location)
    #xml_string = p.read()
    ###print xml_string
    #p.close()
    #product_xml = f.read()
    #init_pose = Pose(Point(0.0,0.0,0.0), Quaternion(0,0,0,0))
    #spawn_urdf_model("robot", product_xml, "robot::base_link", init_pose, world_frame)       
    #init_robot_pose = Pose(Point(0,0,1.0), Quaternion(0.0,0,0,0))
    #init_robot_twist = Twist(Vector3(0,0,0), Vector3(0,0,0))
    #set_model_state(ModelState("robot", init_robot_pose, init_robot_twist, world_frame))
    #cur_model_state = get_model_state("robot",world_frame)
    #link_info = get_link_state("robot::base_link","world")
    #print link_info
    #reset_world()
