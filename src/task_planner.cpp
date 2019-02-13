#include <stdlib.h>
#include <iostream>
#include <time.h>       /* time */
#include <boost/thread.hpp>
//ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <kinect_segmentation/ScanObjectsAction.h>
#include <kinect_segmentation/ScanObjectsGoal.h>

#include <push_vs_grasp/PlanAction.h>
#include <push_vs_grasp/PlanGoal.h>

#include <push_vs_grasp/PickPlaceAction.h>
#include <push_vs_grasp/PickPlaceGoal.h>

void spinThread()
{
  ros::spin();
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "push_client");
  srand (time(NULL));

  actionlib::SimpleActionClient<kinect_segmentation::ScanObjectsAction> ScanObjects_Action_client("scan_objects");
  actionlib::SimpleActionClient<push_vs_grasp::PlanAction> Plan_Action_client("box2d_planner");
  actionlib::SimpleActionClient<push_vs_grasp::PickPlaceAction> PickPlace_Action_client("pick_place");

  boost::thread spin_thread(&spinThread);

  ROS_INFO("Waiting for action Client to startup.");
  ScanObjects_Action_client.waitForServer(); //will wait for infinite time
  ROS_INFO("ScanObjects_Action_client started");
  PickPlace_Action_client.waitForServer(); //will wait for infinite time
  ROS_INFO("PickPlace_Action_client started");
  Plan_Action_client.waitForServer(); //will wait for infinite time
  ROS_INFO("Plan_Action_client started");

  bool plan_success = false;

  while(true)
    {
      //Scan Action
      kinect_segmentation::ScanObjectsGoal ScanObjects_goal;
      ScanObjects_goal.goal_reached = plan_success;

      ScanObjects_Action_client.sendGoal(ScanObjects_goal);
      ScanObjects_Action_client.waitForResult();
      kinect_segmentation::ScanObjectsResult Scan_Result = *ScanObjects_Action_client.getResult();
      actionlib::SimpleClientGoalState scan_state = ScanObjects_Action_client.getState();
      ROS_INFO("Scan Action finished: %s",scan_state.toString().c_str());

      if (scan_state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
	  if (Scan_Result.centroids.empty())
	    {
	      ROS_INFO("Table is EMPTY!");
	      return 0;
	    }
	}
      
      //Plan Action
      push_vs_grasp::PlanGoal planner_goal;
      planner_goal.centroids = Scan_Result.centroids;
      planner_goal.radiuses = Scan_Result.radiuses;
      planner_goal.goal_radiuses = Scan_Result.goal_radiuses;
      planner_goal.colors = Scan_Result.colors;
      planner_goal.red_goal = Scan_Result.red_goal;
      planner_goal.blue_goal = Scan_Result.blue_goal;
      Plan_Action_client.sendGoal(planner_goal);
      bool plan_result = Plan_Action_client.waitForResult();
      push_vs_grasp::PlanResult Plan_Result = *Plan_Action_client.getResult();
      plan_success = Plan_Result.goal_reached;
      actionlib::SimpleClientGoalState plan_state = Plan_Action_client.getState();
      ROS_INFO("Plan Action finished: %s",plan_state.toString().c_str());

      if (Plan_Result.goal_reached) {
	ROS_INFO("Goal Reached..");
	continue;
      }
      
      //PickPlace Action
      push_vs_grasp::PickPlaceGoal pick_place_goal;
      pick_place_goal.action_type = Plan_Result.action_type;
      pick_place_goal.obj_centroid = Plan_Result.obj_centroid;
      pick_place_goal.placement = Plan_Result.placement;
      PickPlace_Action_client.sendGoal(pick_place_goal);
      bool pick_place_result = PickPlace_Action_client.waitForResult();
      actionlib::SimpleClientGoalState pick_place_state = PickPlace_Action_client.getState();
      ROS_INFO("PickPlace Action finished: %s",pick_place_state.toString().c_str());      
    }
  return 0;
}
