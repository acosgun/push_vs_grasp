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
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
import time
#CUSTOM
from push_vs_grasp.msg import RealPickPlaceAction

#GLOBAL VARIABLES
gripperOffset = 0.09

class RealPickPlaceServer:
  def __init__(self):
    self.init_moveit()
    self.Grip = Robotiq2FGripper_robot_output()
    self.home_pose = geometry_msgs.msg.Pose()
    self.go_home()
    self.server = actionlib.SimpleActionServer('real_pick_place', RealPickPlaceAction, self.executeCB, False)
    self.server.start()    
    self.pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', Robotiq2FGripper_robot_output, queue_size=10)
    self.init_gripper()
    rospy.loginfo("Real Pick Place Server ON")

  def move_arm_overhead(self, x, y, z):
    group = self.group
    pose_goal = self.home_pose
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z + gripperOffset
    #quat = quaternion_from_euler(0, 1.57, 0)
    #pose_goal.orientation.x = quat[0]
    #pose_goal.orientation.y = quat[1]
    #pose_goal.orientation.z = quat[2]
    #pose_goal.orientation.w = quat[3]

    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    group.stop()     # Calling `stop()` ensures that there is no residual movement
    group.clear_pose_targets()

  def init_gripper(self):
    self.Grip.rACT = 0
    self.Grip.rPR = 0
    self.Grip.rGTO = 0
    self.Grip.rSP  = 0
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

  def Gripper_close(self):
    self.Grip.rPR = 255
    self.pub.publish(self.Grip)
    time.sleep(2)

  def Gripper_open(self):
    self.Grip.rPR = 0
    self.pub.publish(self.Grip)
    time.sleep(2)
    
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
    p.orientation.w = 1.0
    scene.add_box("table", p, (0.5, 1.5, 0.6))
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
    
  def go_home(self):
    group = self.group
    self.home_pose = group.get_current_pose().pose
    print("W:",self.home_pose.orientation.w)
    print("X:",self.home_pose.orientation.x)
    print("Y:",self.home_pose.orientation.y)
    print("Z:",self.home_pose.orientation.z)

    self.home_pose.position.x = -0.4
    self.home_pose.position.y = 0.1
    self.home_pose.position.z = 0.4
    self.home_pose.orientation.w = 0
    self.home_pose.orientation.x = -0.707106781
    self.home_pose.orientation.y = 0
    self.home_pose.orientation.z = 0.707106781

    group.set_pose_target(self.home_pose)
    plan = group.go(wait=True)
    
  def executeCB(self, goal):
    rospy.loginfo("executeCB: RealPickPlaceAction")

    self.go_home()
    self.Gripper_open()

    ## Send Arm over the Object with some z offset
    x = goal.obj_centroid.point.x
    y = goal.obj_centroid.point.y
    z = goal.obj_centroid.point.z + 0.2
    self.move_arm_overhead(x,y,z)

    self.Gripper_close()

    #Find placement position, move the arm there (random for now)
    min_x = -0.4
    max_x = -0.2
    min_y = -0.3
    max_y = 0.3
    import random
    x = random.uniform(min_x,max_x)
    y = random.uniform(min_y,max_y)
    z = 0.3
    self.move_arm_overhead(x,y,z)
    
    self.Gripper_open()

    self.go_home()
    
    self.server.set_succeeded()
    
if __name__ == '__main__':
  rospy.init_node('real_pick_place_server')

  server = RealPickPlaceServer()
  rospy.spin()
