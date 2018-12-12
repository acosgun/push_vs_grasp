#! /usr/bin/env python

import roslib
roslib.load_manifest('push_vs_grasp')
import rospy
import actionlib

from push_vs_grasp.msg import SimPickPlaceAction, SimPickPlaceGoal
from kinect_segmentation.msg import ScanObjectsAction

if __name__ == '__main__':
    rospy.init_node('task_planner')

    sim_pick_place_client = actionlib.SimpleActionClient('sim_pick_place', SimPickPlaceAction)
    sim_pick_place_client.wait_for_server()
    scan_objects_client = actionlib.SimpleActionClient('scan_objects', ScanObjectsAction)
    scan_objects_client.wait_for_server()

    goal = SimPickPlaceGoal()

    sim_pick_place_client.send_goal(goal)

    if sim_pick_place_client.wait_for_result(rospy.Duration.from_sec(5.0)):
        print sim_pick_place_client.get_result()
    else:
        sim_pick_place_client.cancel_all_goals()
        print('        action timed-out')
