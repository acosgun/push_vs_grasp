#! /usr/bin/env python

#ROS
import roslib
roslib.load_manifest('push_vs_grasp')
import copy
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
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import *
import time
#CUSTOM
from push_vs_grasp.msg import PickPlaceAction

#GLOBAL VARIABLES
gripperOffset = 0.25

class PickPlaceServer:
  def __init__(self):
    #Gazebo Simluation
    rospy.wait_for_service("gazebo/get_model_state")
    rospy.wait_for_service("gazebo/set_model_state")
    self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    self.obj_name =" "
    self.sim = rospy.get_param('~sim')
    self.init_moveit()

    self.Grip = Robotiq2FGripper_robot_output()

    self.home_pose = geometry_msgs.msg.Pose()
    self.Goal_pose = geometry_msgs.msg.Pose()
    self.Target_pose = geometry_msgs.msg.Pose()
    self.obj_state = 'n/a'

    self.server = actionlib.SimpleActionServer('pick_place', PickPlaceAction, self.executeCB, False)
    self.pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', Robotiq2FGripper_robot_output, queue_size=10)

    self.init_home_pose()
    self.init_gripper()

    self.go_home()
    self.server.start()    
    rospy.loginfo("Pick Place Server ON")

  def vanish_gazebo_object(self, obj_name):
    state = ModelState()
    state.model_name = obj_name
    pose = Pose()
    pose.position.x = 10.0
    pose.position.y = 10.0
    pose.position.z = 10.0
    state.pose = pose
    self.set_model_state(state)

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
    self.obj_name = final_name
    self.obj_state = final_response

  def place_object(self, obj_name):
    state = ModelState()
    state.model_name = obj_name
    state.reference_frame = "robot::base_link"

    pose = self.obj_state.pose

    pose.position.x = self.Goal_pose.position.x
    pose.position.y = self.Goal_pose.position.y
    pose.position.z += 0.01

    state.pose = pose
    self.set_model_state(state)

  def init_gripper(self):
    time.sleep(2)
    self.Grip.rACT = 0
    self.Grip.rPR = 0
    self.Grip.rGTO = 0
    self.Grip.rSP  = 255
    self.Grip.rFR = 0
    self.Grip.rATR = 0
    self.pub.publish(self.Grip)
    time.sleep(2)

    self.Grip.rACT = 1
    self.Grip.rPR = 0
    self.Grip.rGTO = 1
    self.Grip.rSP  = 255
    self.Grip.rFR = 150
    self.Grip.rATR = 0
    self.pub.publish(self.Grip)
    time.sleep(2)

  def init_home_pose(self):
    self.home_pose.position.x = -0.4
    self.home_pose.position.y = 0.1
    self.home_pose.position.z = 0.5
    self.home_pose.orientation.w = 0
    self.home_pose.orientation.x = -0.707106781
    self.home_pose.orientation.y = 0
    self.home_pose.orientation.z = 0.707106781

    self.Target_pose = copy.deepcopy(self.home_pose)
    self.Goal_pose   = copy.deepcopy(self.home_pose)

  def init_moveit(self):
    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface("base_link")
    time.sleep(2)
    # Create table obstacle
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    print p.header.frame_id
    p.pose.position.x = -0.5
    p.pose.position.y = 0
    p.pose.position.z = -0.03
    scene.add_box("table", p, (2, 2, 0.06))
    time.sleep(2)

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

  def Gripper_close(self):
    self.Grip.rPR = 105
    self.pub.publish(self.Grip)
    time.sleep(2)

  def Gripper_open(self):
    self.Grip.rPR = 0
    self.pub.publish(self.Grip)
    time.sleep(2)
    
  def Cartesian_To_Pick(self):
    print("Pick")
    if not self.sim:
        self.Gripper_open()

    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose

    wpose.position.x = self.Target_pose.position.x
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y = self.Target_pose.position.y
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = gripperOffset
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.05,        # eef_step
                                       0.0) # jump_threshold

    success = False
    # Plan the trajectory
    if (fraction == 1):
      success = True
      group.execute(plan)
      time.sleep(1)

      if not self.sim:
          self.Gripper_close()
      else:
        #Vanish Object in Gazebo
        self.vanish_gazebo_object(self.obj_name)



    wpose.position.z = 0.4
    group.set_pose_target(self.home_pose)

    plan = group.go(wait=True)

    return success

  def Cartesian_To_Place(self):
    print("Place")
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose

    wpose.position.z = 0.4
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = self.Goal_pose.position.x
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y = self.Goal_pose.position.y
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = gripperOffset
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.05,        # eef_step
                                       0.0) # jump_threshold

    success = False
    # Plan the trajectory
    if (fraction == 1):
      success = True
      group.execute(plan)
      time.sleep(1)

      if not self.sim:
        self.Gripper_open()
      else:
        #Drop Object
        self.place_object(self.obj_name)



    wpose.position.z = 0.4
    group.set_pose_target(self.home_pose)

    plan = group.go(wait=True)

    return success


  def go_home(self):
    print("Home")
    print("x", self.home_pose.position.x)
    print("y", self.home_pose.position.y)

    group = self.group
   
    waypoints = []
    wpose = group.get_current_pose().pose

    wpose.position.y = self.home_pose.position.y
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = self.home_pose.position.x
    waypoints.append(copy.deepcopy(wpose))

    wpose = self.home_pose
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.05,        # eef_step
                                       0.0) # jump_threshold

    success = False
    print(fraction)

    # Plan the trajectory
    if (fraction == 1):
      success = True
      print("Home")

      group.execute(plan)
      time.sleep(1)

    return success

    
  def executeCB(self, goal):
    rospy.loginfo("executeCB: PickPlaceAction")

    ## Send Arm over the Object with some z offset
    self.Target_pose.position.x = goal.obj_centroid.point.x
    self.Target_pose.position.y = goal.obj_centroid.point.y

    if(self.sim):
      #Associate the centroid with one of the cylindrical objects in Gazebo
      self.get_closest_object(goal.obj_centroid.point)

    success = self.Cartesian_To_Pick()


    if(success):
      #Find placement position, move the arm there (random for now)
      min_x = -0.5
      max_x = -0.4
      min_y = -0.3
      max_y = 0.3
      import random
      x = random.uniform(min_x,max_x)
      y = random.uniform(min_y,max_y)
      z = 0.2

      self.Goal_pose.position.x = x
      self.Goal_pose.position.y = y
      self.Goal_pose.position.z = z

      success = self.Cartesian_To_Place()

      
      success = self.go_home()
    
    self.server.set_succeeded()
    
if __name__ == '__main__':
  rospy.init_node('pick_place_server')

  server = PickPlaceServer()
  rospy.spin()
