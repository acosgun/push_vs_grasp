#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <time.h>
#include <boost/thread.hpp>
#include <fstream>

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

#include <push_vs_grasp/GenerateCylindersAction.h>
#include <push_vs_grasp/GenerateCylindersGoal.h>

void spinThread()
{
  ros::spin();
}

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "push_client");

  ros::NodeHandle nh("~");
  double obj_x_min; double obj_x_max; double obj_y_min; double obj_y_max;
  double goal_x_min; double goal_x_max; double goal_y_min; double goal_y_max;
  double red_radius; double blue_radius; double object_radius;
  int num_obj_min; int num_obj_max; int runs_per_obj; int num_algos;
  double timeout_secs; bool perfect_perception; bool sim; 
  std::string log_folder; bool enable_log; std::ofstream myfile; std::string concat;
  
  nh.getParam("/perfect_perception", perfect_perception);
  nh.getParam("/sim", sim);
  
  nh.getParam("obj_x_min", obj_x_min);
  nh.getParam("obj_x_max", obj_x_max);
  nh.getParam("obj_y_min", obj_y_min);
  nh.getParam("obj_y_max", obj_y_max);

  nh.getParam("goal_x_min", goal_x_min);
  nh.getParam("goal_x_max", goal_x_max);
  nh.getParam("goal_y_min", goal_y_min);
  nh.getParam("goal_y_max", goal_y_max);

  nh.getParam("red_radius", red_radius);
  nh.getParam("blue_radius", blue_radius);
  nh.getParam("object_radius", object_radius);
  
  nh.getParam("num_obj_min", num_obj_min);
  nh.getParam("num_obj_max", num_obj_max);
  nh.getParam("runs_per_obj", runs_per_obj);
  nh.getParam("num_algos", num_algos);
  
  nh.getParam("timeout_secs", timeout_secs);
  nh.getParam("log_folder", log_folder);  
  nh.getParam("enable_log", enable_log);    
  
  srand (time(NULL));
  
  if (enable_log) {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );  
    char buffer [80];
    strftime (buffer,80,"%X-%F",now);
    std::string filename(buffer);
    concat = log_folder + filename + ".csv";
  
    myfile.open(concat, std::ios::in| std::ios::out | std::ios::binary);  // will not create file
    if (!myfile.is_open()){//File doesn't exist yet
      myfile.clear();
      myfile.open(concat, std::ios::out);  // will create if necessary
      myfile << "Algorithm, Number of Objects, Time Elapsed, Number of Pick/Places, Number of Pushes, Successful";    
    }
    myfile.close();
  }

  actionlib::SimpleActionClient<kinect_segmentation::ScanObjectsAction> ScanObjects_Action_client("scan_objects");
  actionlib::SimpleActionClient<push_vs_grasp::PlanAction> Plan_Action_client("box2d_planner");
  actionlib::SimpleActionClient<push_vs_grasp::PickPlaceAction> PickPlace_Action_client("pick_place");
  actionlib::SimpleActionClient<push_vs_grasp::GenerateCylindersAction> GenerateCylinders_Action_client("generate_cylinders");

  boost::thread spin_thread(&spinThread);

  ROS_INFO("Waiting for ScanObjects action client.");
  ScanObjects_Action_client.waitForServer();
  ROS_INFO("ScanObjects_Action_client started");

  ROS_INFO("Waiting for PickPlace action client.");
  PickPlace_Action_client.waitForServer();
  ROS_INFO("PickPlace action client started");

  ROS_INFO("Waiting for Plan action client.");  
  Plan_Action_client.waitForServer();  
  ROS_INFO("Plan action client started");

  ROS_INFO("Waiting for Generate Cylinders action client.");
  GenerateCylinders_Action_client.waitForServer();
  ROS_INFO("Generate Cylinders action client started");  
    
  for (unsigned int num_obj = num_obj_min; num_obj < num_obj_max+1; num_obj++)
    {
      for (unsigned int num_run = 0; num_run < runs_per_obj; num_run++)
	{
	  for (unsigned int algo = 0; algo < num_algos; algo++)
	    {

	      //generate random goals
	      geometry_msgs::PointStamped redGoal_base_link;
	      geometry_msgs::PointStamped blueGoal_base_link;	      
  	      while(true)
		{
		  double x1 = fRand(obj_x_min,obj_x_max);
		  double x2 = fRand(obj_x_min,obj_x_max);
		  double y1 = fRand(obj_y_min,obj_y_max);
		  double y2 = fRand(obj_y_min,obj_y_max);
		  
		  double dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));
		  if (dist < red_radius + blue_radius)
		    {
		      continue;
		    }
		  else
		    {
		      redGoal_base_link.point.x = x1;
		      redGoal_base_link.point.y = y1;
		      blueGoal_base_link.point.x = x2;
		      blueGoal_base_link.point.y = y2;
		      redGoal_base_link.point.z = 0;
		      blueGoal_base_link.point.z = 0;		  
		      break;
		    }
		}

	      
	      //GenerateCylinders Action
	      push_vs_grasp::GenerateCylindersGoal gen_cylinders_goal;
	      gen_cylinders_goal.min_x = obj_x_min;
	      gen_cylinders_goal.max_x = obj_x_max;
	      gen_cylinders_goal.min_y = obj_y_min;
	      gen_cylinders_goal.max_y = obj_y_max;
	      gen_cylinders_goal.num_objs = num_obj;
	      gen_cylinders_goal.red_goal = redGoal_base_link;
	      gen_cylinders_goal.blue_goal = blueGoal_base_link;
	      gen_cylinders_goal.red_radius = red_radius;
	      gen_cylinders_goal.blue_radius = blue_radius;
	      gen_cylinders_goal.get_perfect_perception = false;
	      GenerateCylinders_Action_client.sendGoal(gen_cylinders_goal);  
	      bool gen_cylinders_result = GenerateCylinders_Action_client.waitForResult();  
	      push_vs_grasp::GenerateCylindersResult GenerateCylinders_Result = *GenerateCylinders_Action_client.getResult();
	      actionlib::SimpleClientGoalState gen_state = GenerateCylinders_Action_client.getState();
	      ROS_INFO("Generate Cylinders Action finished: %s", gen_state.toString().c_str());

	      ros::Duration time_elapsed;    
	      ros::Time t_start = ros::Time::now();
	      bool plan_success = false;

	      unsigned int num_pushes = 0;
	      unsigned int num_picks = 0;
	  
	      while(true)
		{
		  
		  kinect_segmentation::ScanObjectsGoal ScanObjects_goal;
		  ScanObjects_goal.red_goal = redGoal_base_link;
		  ScanObjects_goal.blue_goal = blueGoal_base_link;
		  ScanObjects_goal.goal_radiuses.push_back(red_radius);
		  ScanObjects_goal.goal_radiuses.push_back(blue_radius);
		  
		  kinect_segmentation::ScanObjectsResult Scan_Result;
	      
		  if (sim && perfect_perception)
		    {
		      push_vs_grasp::GenerateCylindersGoal perf_perception_goal;
		      perf_perception_goal.get_perfect_perception = true;
		      GenerateCylinders_Action_client.sendGoal(perf_perception_goal);
		      GenerateCylinders_Action_client.waitForResult();		      
		      push_vs_grasp::GenerateCylindersResult perf_perception_result = *GenerateCylinders_Action_client.getResult();

		      Scan_Result.centroids = perf_perception_result.centroids;
		      Scan_Result.radiuses = GenerateCylinders_Result.radiuses;
		      Scan_Result.colors = GenerateCylinders_Result.colors;
		      ROS_INFO("Perfect Perception SUCCEEDED");
		    }
		  else
		    {
		      //Scan Action
		      ScanObjects_Action_client.sendGoal(ScanObjects_goal);
		      ScanObjects_Action_client.waitForResult();
		      Scan_Result = *ScanObjects_Action_client.getResult();
		      actionlib::SimpleClientGoalState scan_state = ScanObjects_Action_client.getState();
		      ROS_INFO("Scan Action finished: %s",scan_state.toString().c_str());
		    }

		  if (Scan_Result.centroids.empty()){
		    ROS_INFO("Table is EMPTY!");
		    return 0;
		  }		      
	      
		  //Plan Action
		  push_vs_grasp::PlanGoal planner_goal;
		  planner_goal.centroids = Scan_Result.centroids;
		  planner_goal.radiuses = Scan_Result.radiuses;
		  planner_goal.goal_radiuses = ScanObjects_goal.goal_radiuses;
		  planner_goal.colors = Scan_Result.colors;
		  planner_goal.red_goal = ScanObjects_goal.red_goal;
		  planner_goal.blue_goal = ScanObjects_goal.blue_goal;
		  planner_goal.algo = algo;
		  Plan_Action_client.sendGoal(planner_goal);
		  bool plan_result = Plan_Action_client.waitForResult();
		  push_vs_grasp::PlanResult Plan_Result = *Plan_Action_client.getResult();
		  plan_success = Plan_Result.goal_reached;
		  actionlib::SimpleClientGoalState plan_state = Plan_Action_client.getState();
		  ROS_INFO("Plan Action finished: %s",plan_state.toString().c_str());

		  break;		  
		  
		  //PickPlace Action
		  push_vs_grasp::PickPlaceGoal pick_place_goal;		  
		  pick_place_goal.action_type = Plan_Result.action_type;
		  pick_place_goal.obj_centroid = Plan_Result.obj_centroid;
		  pick_place_goal.placement = Plan_Result.placement;
		  PickPlace_Action_client.sendGoal(pick_place_goal);

		  bool pick_place_result = PickPlace_Action_client.waitForResult();
		  push_vs_grasp::PickPlaceResult Pick_Place_Result = *PickPlace_Action_client.getResult();
		  actionlib::SimpleClientGoalState pick_place_state = PickPlace_Action_client.getState();
		  ROS_INFO("PickPlace Action finished: %s",pick_place_state.toString().c_str());
		  if (pick_place_state == actionlib::SimpleClientGoalState::SUCCEEDED)
		    {
		      if (pick_place_goal.action_type == 0) {
			num_pushes++;
		      }
		      else if (pick_place_goal.action_type == 1) {
			num_picks++;
		      }
		    }

		  // Time elapsed
		  ros::Time t_end = ros::Time::now();
		  time_elapsed = t_end - t_start;

		  bool timeout = time_elapsed >= ros::Duration(timeout_secs);
	      
		  if (Plan_Result.goal_reached || timeout) {
		
		    //TODO: check actual sim result?
		    bool task_success = false;
		    if (Plan_Result.goal_reached) {
		      task_success = true;
		    }
		  
		    if (enable_log) {		      
		      myfile.open (concat.c_str(), std::fstream::app);		
		      myfile << "\n" << num_obj << "," << num_run << "," << algo << "," << task_success << "," << time_elapsed << "," << num_picks << "," << num_pushes;
		      myfile.close();
		    }
		    std::cout << "--- Run #: ---" << num_run << std::endl;
		    std::cout << "# Objects: " << num_obj << std::endl;		    
		    std::cout << "Algo Choice: " << algo << std::endl;
		    std::cout << " # Pushes: " << num_pushes << std::endl;
		    std::cout << " # Picks : " << num_picks << std::endl;
		    std::cout << "Success: " << task_success << std::endl;
		    std::cout << " Time: " << time_elapsed << std::endl;
		    std::cout << "--- Run #: ---" << num_run << std::endl;
		    break;
		  }
		}
	    }
	}

    }
    
  return 0;
}
