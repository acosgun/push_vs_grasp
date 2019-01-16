#include <stdlib.h>
#include <iostream>
#include <time.h>       /* time */
#include <boost/thread.hpp>
//ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <push_vs_grasp/MoveItPushAction.h>
#include <push_vs_grasp/MoveItPushGoal.h>

#include <kinect_segmentation/ScanObjectsAction.h>
#include <kinect_segmentation/ScanObjectsGoal.h>

#include <push_vs_grasp/PlanAction.h>
#include <push_vs_grasp/PlanGoal.h>

void spinThread()
{
  ros::spin();
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "push_client");
  srand (time(NULL));

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<push_vs_grasp::MoveItPushAction>   Pushing_Action_client("Pushing");
  actionlib::SimpleActionClient<kinect_segmentation::ScanObjectsAction> ScanObjects_Action_client("scan_objects");
  actionlib::SimpleActionClient<push_vs_grasp::PlanAction> Plan_Action_client("box2d_planner");
  
  boost::thread spin_thread(&spinThread);
  
  ROS_INFO("Waiting for action Client to startup.");
  // wait for the action server to start

  ScanObjects_Action_client.waitForServer(); //will wait for infinite time
  ROS_INFO("ScanObjects_Action_client started");

  Pushing_Action_client.waitForServer(); //will wait for infinite time
  ROS_INFO("Pushing_Action_client started");

  Plan_Action_client.waitForServer(); //will wait for infinite time
  ROS_INFO("Plan_Action_client started");

  
  ///*
  // send a goal to the action
  kinect_segmentation::ScanObjectsGoal ScanObjects_goal;
  push_vs_grasp::MoveItPushGoal Push_goal;
while(true){
  ScanObjects_Action_client.sendGoal(ScanObjects_goal);

  //wait for the action to return
  bool finished_before_timeout = ScanObjects_Action_client.waitForResult(ros::Duration(10.0));

  if (finished_before_timeout)
  {
      ROS_INFO("Action finished");
      kinect_segmentation::ScanObjectsResult Scan_Result = *ScanObjects_Action_client.getResult();
      Push_goal.all_centroids = Scan_Result.centroids;
      std::vector<geometry_msgs::PointStamped> Centroids = Scan_Result.centroids; 
      geometry_msgs::PointStamped goalXYZ ;
        //Randomly choose a point from the array
      int Centroid_indx = rand() % (static_cast<int>(Centroids.size()) + 1);
      goalXYZ.point.x = Centroids[Centroid_indx].point.x;
      goalXYZ.point.y = Centroids[Centroid_indx].point.y;
      goalXYZ.point.z = Centroids[Centroid_indx].point.z;

      Push_goal.obj_centroid  = goalXYZ;
      Pushing_Action_client.sendGoal(Push_goal);
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  //exit

sleep(0.1);
}
//*/
  ros::shutdown();
  spin_thread.join();
  return 0;
}
