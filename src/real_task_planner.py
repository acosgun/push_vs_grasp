#! /usr/bin/env python

#ROS
import roslib
roslib.load_manifest('push_vs_grasp')
import rospy
import actionlib
from geometry_msgs.msg import PointStamped

#CUSTOM
from push_vs_grasp.msg import RealPickPlaceAction, RealPickPlaceGoal
from kinect_segmentation.msg import ScanObjectsAction, ScanObjectsGoal

if __name__ == '__main__':
    rospy.init_node('task_planner')
    
    real_pick_place_client = actionlib.SimpleActionClient('real_pick_place', RealPickPlaceAction)
    real_pick_place_client.wait_for_server()
    rospy.loginfo('Task Planner: RealPickPlaceAction ON')
    
    scan_objects_client = actionlib.SimpleActionClient('scan_objects', ScanObjectsAction)
    scan_objects_client.wait_for_server()
    rospy.loginfo('Task Planner: ScanObjectsAction ON')

    rospy.loginfo("Task Planner: All Actionlib Servers ON")


    while True:        
        scan_goal = ScanObjectsGoal()
        scan_objects_client.send_goal(scan_goal)
        
        if not scan_objects_client.wait_for_result(rospy.Duration.from_sec(5.0)):
            scan_objects_client.cancel_all_goals()
            print('ScanObjectsAction timed-out')
            
        centroids = scan_objects_client.get_result().centroids
        rospy.loginfo('Num Objects: ' + str(len(centroids)))

        if len(centroids):
            pick_place_goal = RealPickPlaceGoal()
            from random import randint
            random_obj_index = randint(0, len(centroids) - 1)
            pick_place_goal.obj_centroid = centroids[random_obj_index]
            real_pick_place_client.send_goal(pick_place_goal)

            if not real_pick_place_client.wait_for_result(rospy.Duration.from_sec(5.0)):
                real_pick_place_client.cancel_all_goals()
                print('RealPickPlaceAction timed-out')
                
            print real_pick_place_client.get_result()
