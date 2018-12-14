#! /usr/bin/env python

#ROS
import roslib
roslib.load_manifest('push_vs_grasp')
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

roslib.load_manifest("ur_kinematics")
from ur_kin_py import forward, inverse

#CUSTOM
from push_vs_grasp.msg import SimPickPlaceAction

class SimPickPlaceServer:
  def __init__(self):
    self.init_moveit()
    self.go_home()
    #self.server = actionlib.SimpleActionServer('sim_pick_place', SimPickPlaceAction, self.executeCB, False)
    #self.server.start()    
    rospy.loginfo("Sim Pick Place Server ON")

  def init_moveit(self):
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
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
    joint_goal[1] = -0.5*pi
    joint_goal[2] = -0.7*pi
    joint_goal[3] = -0.3*pi
    joint_goal[4] = 0.5*pi
    joint_goal[5] = 0
    group.go(joint_goal, wait=True)
    group.stop()
    
  def executeCB(self, goal):
    rospy.loginfo("executeCB: SimPickPlaceAction")

    go_home()


    #IK. Send Arm over the Object with some z offset
    #    x = goal.obj_centroid
    
    #Associate the centroid with one of the cylindrical objects in Gazebo
    
    #Vanish Object in Gazebo

    #Plan & Move the arm to a random location

    #Spawn Object in Gazebo

    #Go to base

    self.server.set_succeeded()
    
if __name__ == '__main__':
  rospy.init_node('sim_pick_place_server')
  server = SimPickPlaceServer()
  rospy.spin()
