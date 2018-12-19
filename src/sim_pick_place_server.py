#! /usr/bin/env python

#ROS
import roslib
roslib.load_manifest('push_vs_grasp')
import rospy
import actionlib
from moveit_python import *
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import *

#CUSTOM
from push_vs_grasp.msg import SimPickPlaceAction

#GLOBAL VARIABLES
gripperOffset = 0.09

class SimPickPlaceServer:
  def __init__(self):
    rospy.wait_for_service("gazebo/get_model_state")
    rospy.wait_for_service("gazebo/set_model_state")
    self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    self.init_moveit()
    self.go_home()
    self.server = actionlib.SimpleActionServer('sim_pick_place', SimPickPlaceAction, self.executeCB, False)
    self.server.start()    
    rospy.loginfo("Sim Pick Place Server ON")
    
  def vanish_gazebo_object(self, obj_name):
    state = ModelState()
    state.model_name = obj_name
    pose = Pose()
    pose.position.x = 10.0
    pose.position.y = 10.0
    pose.position.z = 10.0
    state.pose = pose
    self.set_model_state(state)

  def place_object(self, obj_name, obj_state, x, y, z):
    state = ModelState()
    state.model_name = obj_name
    state.reference_frame = "robot::base_link"
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = obj_state.pose.position.z + 0.01
    state.pose = pose
    self.set_model_state(state)

  def move_arm_overhead(self, x, y, z):
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z + gripperOffset
    quat = quaternion_from_euler(0, 1.57, 0)
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    group.stop()     # Calling `stop()` ensures that there is no residual movement
    group.clear_pose_targets()
    
  def get_closest_object(self, closest_obj):
    max_num_objs = 5
    model_name = "unit_cylinder"
    import sys
    min_dist = sys.float_info.max
    final_name = 'n/a'
    final_response = 'n/a'  
    for i in range(max_num_objs):          
      cur_obj_name = model_name + str(i)
      response = self.get_model_state(cur_obj_name,  "robot::base_link")
      if response.success:
        import numpy as np
        closest_obj_pos = np.array([closest_obj.x, closest_obj.y, closest_obj.z])
        response_obj_pos =np.array([response.pose.position.x, response.pose.position.y, response.pose.position.z])       
        dist = np.linalg.norm(closest_obj_pos - response_obj_pos)
        if dist < min_dist:
          min_dist = dist
          final_response = response
          final_name = cur_obj_name
    return final_name, final_response
        
    
  def init_moveit(self):
    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface("base_link")

    # Create table obstacle
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    print p.header.frame_id
    p.pose.position.x = 0.42
    p.pose.position.y = -0.2
    p.pose.position.z = 0.3
    scene.add_box("table", p, (0.5, 1.5, 0.6))
    
    group_name = "manipulator"
    self.group_name = group_name
    group = moveit_commander.MoveGroupCommander(self.group_name)
    planning_frame = group.get_planning_frame()
    #print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    #print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""
    self.group = group
    
  def go_home(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -0.4*pi
    joint_goal[2] = -0.7*pi
    joint_goal[3] = -0.4*pi
    joint_goal[4] = 0.5*pi
    joint_goal[5] = 0
    group.go(joint_goal, wait=True)
    group.stop()
    
  def executeCB(self, goal):
    rospy.loginfo("executeCB: SimPickPlaceAction")

    self.go_home()
    
    ## Send Arm over the Object with some z offset
    x = goal.obj_centroid.point.x
    y = goal.obj_centroid.point.y
    z = goal.obj_centroid.point.z + 0.1
    self.move_arm_overhead(x,y,z)
    
    #Associate the centroid with one of the cylindrical objects in Gazebo
    [obj_name, obj_state] = self.get_closest_object(goal.obj_centroid.point)

    #Vanish Object in Gazebo
    self.vanish_gazebo_object(obj_name)

    #Find placement position, move the arm there (random for now)
    min_x = -0.4
    max_x = -0.2
    min_y = -0.3
    max_y = 0.3
    import random
    x = random.uniform(min_x,max_x)
    y = random.uniform(min_y,max_y)
    z = obj_state.pose.position.z + 0.1
    self.move_arm_overhead(x,y,z)
    
    #Drop Object
    self.place_object(obj_name, obj_state, x, y, z)
    
    self.go_home()
    
    self.server.set_succeeded()
    
if __name__ == '__main__':
  rospy.init_node('sim_pick_place_server')
  server = SimPickPlaceServer()
  rospy.spin()
