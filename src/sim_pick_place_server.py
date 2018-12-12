#! /usr/bin/env python

import roslib
roslib.load_manifest('push_vs_grasp')
import rospy
import actionlib

from push_vs_grasp.msg import SimPickPlaceAction

class SimPickPlaceServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('sim_pick_place', SimPickPlaceAction, self.execute, False)
    self.server.start()
    rospy.loginfo("Sim Pick Place Server ON")
    
  def execute(self, goal):
      #

      
    self.server.set_succeeded()
    

if __name__ == '__main__':
  rospy.init_node('sim_pick_place_server')
  server = SimPickPlaceServer()
  rospy.spin()
