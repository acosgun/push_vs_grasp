#include <stdlib.h>
#include <iostream>
#include <time.h>       /* time */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <push_vs_grasp/MoveItPushAction.h>
#include <kinect_segmentation/ScanObjectsAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "Push_Obj_client");
  srand (time(NULL));

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<push_vs_grasp::MoveItPushAction>   Pushing_Action("pushing",true);
  actionlib::SimpleActionClient<kinect_segmentation::ScanObjectsAction> ScanObjects_Action("scanobjects",true);
  ROS_INFO("Waiting for action Client to startup.");
  // wait for the action server to start

  Pushing_Action.waitForServer(); //will wait for infinite time
  ROS_INFO("Action Client started, sending goal.");

  /*
  ScanObjects_Action.waitForServer(); //will wait for infinite time
 
  ROS_INFO("Action Client started, sending goal.");

  // send a goal to the action
  kinect_segmentation::ScanObjectsGoal ScanObjects_goal;
  push_vs_grasp::MoveItPushGoal Push_goal;

  ScanObjects_Action.sendGoal(ScanObjects_goal);

  //wait for the action to return
  bool finished_before_timeout = ScanObjects_Action.waitForResult(ros::Duration(10.0));

  if (finished_before_timeout)
  {
      ROS_INFO("Action finished");
      kinect_segmentation::ScanObjectsResult Scan_Result = *ScanObjects_Action.getResult();
      Push_goal.all_centroids = Scan_Result.centroids;
      std::vector<geometry_msgs::PointStamped> Centroids = Scan_Result.centroids; 
      geometry_msgs::PointStamped goalXYZ ;
        //Randomly choose a point from the array
      int Centroid_indx = rand() % (static_cast<int>(Centroids.size()) + 1);
      goalXYZ.point.x = Centroids[Centroid_indx].point.x;
      goalXYZ.point.y = Centroids[Centroid_indx].point.y;
      goalXYZ.point.z = Centroids[Centroid_indx].point.z;

      Push_goal.obj_centroid  = goalXYZ;
      Pushing_Action.sendGoal(Push_goal);
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  //exit
*/
  ros::spin();
  return 0;
}