#! /usr/bin/env python

#ROS
import roslib
roslib.load_manifest('push_vs_grasp')
import rospy
import actionlib
from geometry_msgs.msg import PointStamped

#CUSTOM
from push_vs_grasp.msg import SimPickPlaceAction, SimPickPlaceGoal
from kinect_segmentation.msg import ScanObjectsAction, ScanObjectsGoal

if __name__ == '__main__':
    rospy.init_node('task_planner')

    #sim_pick_place_client = actionlib.SimpleActionClient('sim_pick_place', SimPickPlaceAction)
    #sim_pick_place_client.wait_for_server()
    scan_objects_client = actionlib.SimpleActionClient('scan_objects', ScanObjectsAction)
    scan_objects_client.wait_for_server()

    scan_goal = ScanObjectsGoal()
    scan_objects_client.send_goal(scan_goal)

    if not scan_objects_client.wait_for_result(rospy.Duration.from_sec(5.0)):
        scan_objects_client.cancel_all_goals()
        print('ScanObjectsAction timed-out')

    centroids = scan_objects_client.get_result().centroids
    print ('Num Objects: ' + str(len(centroids)))

    

    #goal = SimPickPlaceGoal()
    #sim_pick_place_client.send_goal(goal)    
    #if sim_pick_place_client.wait_for_result(rospy.Duration.from_sec(5.0)):
    #    print sim_pick_place_client.get_result()
    #else:
    #    sim_pick_place_client.cancel_all_goals()
    #    print('        action timed-out')
