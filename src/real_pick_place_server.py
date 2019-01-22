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
import time
#CUSTOM
from push_vs_grasp.msg import RealPickPlaceAction

#GLOBAL VARIABLES
gripperOffset = 0.25

class RealPickPlaceServer:
  def __init__(self):
    self.init_moveit()
    self.Grip = Robotiq2FGripper_robot_output()
    self.home_pose = geometry_msgs.msg.Pose()
    self.Goal_pose = geometry_msgs.msg.Pose()
    self.Target_pose = geometry_msgs.msg.Pose()

    self.server = actionlib.SimpleActionServer('real_pick_place', RealPickPlaceAction, self.executeCB, False)
    self.pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', Robotiq2FGripper_robot_output, queue_size=10)

    self.init_home_pose()
    self.init_gripper()

    self.go_home()
    self.server.start()    
    rospy.loginfo("Real Pick Place Server ON")

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
      self.Gripper_close()



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
      self.Gripper_open()



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
    rospy.loginfo("executeCB: RealPickPlaceAction")

    #success = self.go_home()

    ## Send Arm over the Object with some z offset
    self.Target_pose.position.x = goal.obj_centroid.point.x
    self.Target_pose.position.y = goal.obj_centroid.point.y

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
  rospy.init_node('real_pick_place_server')

  server = RealPickPlaceServer()
  rospy.spin()
