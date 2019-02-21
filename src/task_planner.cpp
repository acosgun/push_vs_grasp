#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <time.h>       /* time */
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

int main (int argc, char **argv)
{
  ros::init(argc, argv, "push_client");

  ros::NodeHandle nh("~");
  double obj_x_min; double obj_x_max; double obj_y_min; double obj_y_max;
  double goal_x_min; double goal_x_max; double goal_y_min; double goal_y_max;
  double red_radius; double blue_radius; double object_radius;
  int num_obj_min; int num_obj_max; int runs_per_obj;
  double timeout_secs; bool perfect_perception; bool sim;
  std::string log_folder;
  
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

  nh.getParam("timeout_secs", timeout_secs);
  nh.getParam("log_folder", log_folder);  

  srand (time(NULL));

  // File for logging data
  std::ofstream myfile;
  time_t t = time(0);   // get time now
  struct tm * now = localtime( & t );  
  char buffer [80];
  strftime (buffer,80,"%X-%F",now);
  std::string filename(buffer);
  std::string concat = log_folder + filename + ".csv";
  std::cout << concat  <<std::endl;
  
  myfile.open(concat, std::ios::in| std::ios::out | std::ios::binary);  // will not create file
  if (!myfile.is_open()){//File doesn't exist yet
    myfile.clear();
    myfile.open(concat, std::ios::out);  // will create if necessary
    myfile << "Algorithm, Number of Objects, Time Elapsed, Number of Pick/Places, Number of Pushes, Successful";    
  }
  myfile.close();

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
  
  // Randomly generate goal regions and objects
  // Generation of goal regions
  geometry_msgs::PointStamped redGoal_base_link;
  geometry_msgs::PointStamped blueGoal_base_link;

  redGoal_base_link.header.frame_id = "base_link";
  blueGoal_base_link.header.frame_id = "base_link";

  double x1, x2, y1, y2;
  std::random_device rd;
  std::mt19937 gen(rd());
  // Distribution encompasses region projected by projector
  // Vertical Limits
  std::uniform_real_distribution<> dist1(goal_y_min, goal_y_max); // Need to make table bigger in box2d and gazebo //REAL: (-0.17,-0.85);
  // Horizontal Limits
  std::uniform_real_distribution<> dist2(goal_x_min, goal_x_max); // REAL: (-0.465, 0.75)
  
  for (unsigned int num_obj = num_obj_min; num_obj < num_obj_max; num_obj++)
    {

      for (unsigned int num_run = 0; num_run < runs_per_obj; num_run++)
	{
	  // Random generation of goal regions
	  bool goalCheck = 1;

	  while(goalCheck){
	    x1 = dist1(gen);
	    x2 = dist1(gen);
	    y1 = dist2(gen);
	    y2 = dist2(gen);

	    if(((x2 > (x1+red_radius)) || (x2 < (x1-red_radius))) && ((y2 > (y1+red_radius)) || (y2 < (y1-red_radius)))){
	      goalCheck = 0;
	    }
	  }

	  redGoal_base_link.point.x = x1;
	  blueGoal_base_link.point.x = x2;
	  redGoal_base_link.point.y = y1;
	  blueGoal_base_link.point.y = y2;
	  redGoal_base_link.point.z = 0;
	  blueGoal_base_link.point.z = 0;


	  // If perfect perception, read the positions of the objects directly rather than through perception
    
	  /*
	  // Random generation of objects
	  // Randomly choosing number of objects to spawn (between 1 and 10)
	  bool objectCheck = true;
	  float x1, y1;
	  int numObjects =  (rand() % 10) + 1; // randomly generating number between 1 and 10
	  std::vector<float> x_vals;
	  std::vector<float> y_vals;
	  std::vector<std::string> colors;

	  // Determining colours of objects
	  for(int i = 0; i < numObjects; i++){
	  // Red = 0, Blue = 1
	  if(rand()%2){
	  colors.push_back("red");
	  }
	  else{
	  colors.push_back("blue");
	  }
	  }

	  // Determining positions of objects
	  std::vector<geometry_msgs::PointStamped> object_centroids;
	  geometry_msgs::PointStamped temp;

	  temp.point.x = dist1(gen);
	  temp.point.y = dist2(gen);

	  object_centroids.push_back(temp);

	  while(objectCheck && object_centroids.size() < numObjects){ // Positions of objects
	  x1 = dist1(gen);
	  y1 = dist2(gen);

      
	  // Checking objects aren't overlapping
	  for(int i = 0; i < object_centroids.size(); i++){
	  // Ensuring collision free placement of objects
	  if(x1 > object_centroids[i].point.x + 2*object_radius || x1 < object_centroids[i].point.x - 2*object_radius || y1 < object_centroids[i].point.y - 2*object_radius || y1 > object_centroids[i].point.y + 2*object_radius){
	  if(i == object_centroids.size() - 1){ // If no collisions with other objects
            
	  temp.point.x = x1;
	  temp.point.y = y1;
	  object_centroids.push_back(temp);  
	  }
	  }
	  else{
	  break;
	  }

	  if(i >= numObjects - 1){ // if the position of all the objects have been successfully determined
	  objectCheck = false;
	  break;
	  }
	  }

	  }
	  */

	  //GenerateCylinders Action
	  push_vs_grasp::GenerateCylindersGoal gen_cylinders_goal;
	  gen_cylinders_goal.min_x = obj_x_min;
	  gen_cylinders_goal.max_x = obj_x_max;
	  gen_cylinders_goal.min_y = obj_y_min;
	  gen_cylinders_goal.max_y = obj_y_max;

	  gen_cylinders_goal.num_objs = num_obj;
	  GenerateCylinders_Action_client.sendGoal(gen_cylinders_goal);  
	  bool gen_cylinders_result = GenerateCylinders_Action_client.waitForResult();
  
	  push_vs_grasp::GenerateCylindersResult GenerateCylinders_Result = *GenerateCylinders_Action_client.getResult();
	  actionlib::SimpleClientGoalState gen_state = GenerateCylinders_Action_client.getState();
	  ROS_INFO("Generate Cylinders Action finished: %s", gen_state.toString().c_str());  

	  ros::Duration time_elapsed;    
	  ros::Time t_start = ros::Time::now();
	  bool plan_success = false;
    
	  while(true)
	    {
	      //Scan Action
	      kinect_segmentation::ScanObjectsGoal ScanObjects_goal;
	      ScanObjects_goal.goal_reached = plan_success;
	      ScanObjects_goal.red_goal = redGoal_base_link;
	      ScanObjects_goal.blue_goal = blueGoal_base_link;
	      ScanObjects_goal.goal_radiuses.push_back(red_radius);
	      ScanObjects_goal.goal_radiuses.push_back(blue_radius);

	      ScanObjects_Action_client.sendGoal(ScanObjects_goal);
	      ScanObjects_Action_client.waitForResult();
	      kinect_segmentation::ScanObjectsResult Scan_Result = *ScanObjects_Action_client.getResult();
	      actionlib::SimpleClientGoalState scan_state = ScanObjects_Action_client.getState();
	      ROS_INFO("Scan Action finished: %s",scan_state.toString().c_str());

	      if (scan_state == actionlib::SimpleClientGoalState::SUCCEEDED){
		if (Scan_Result.centroids.empty()){
		  ROS_INFO("Table is EMPTY!");
		  return 0;
		}
	      }
      
	      //Plan Action

	      /*
		if(sim){
		planner_goal.centroids = Scan_Result.centroids;
		planner_goal.radiuses = Scan_Result.radiuses;
		planner_goal.colors = colors;
		}
		else{
		planner_goal.centroids = Scan_Result.centroids;
		planner_goal.radiuses = Scan_Result.radiuses;
		planner_goal.colors = Scan_Result.colors;
      
		}


	      */

	      push_vs_grasp::PlanGoal planner_goal;
	      planner_goal.centroids = Scan_Result.centroids;
	      planner_goal.radiuses = Scan_Result.radiuses;
	      planner_goal.goal_radiuses = ScanObjects_goal.goal_radiuses;
	      planner_goal.colors = Scan_Result.colors;
	      planner_goal.red_goal = ScanObjects_goal.red_goal;
	      planner_goal.blue_goal = ScanObjects_goal.blue_goal;
	      Plan_Action_client.sendGoal(planner_goal);
	      bool plan_result = Plan_Action_client.waitForResult();
	      push_vs_grasp::PlanResult Plan_Result = *Plan_Action_client.getResult();
	      plan_success = Plan_Result.goal_reached;
	      actionlib::SimpleClientGoalState plan_state = Plan_Action_client.getState();
	      ROS_INFO("Plan Action finished: %s",plan_state.toString().c_str());

      
      
	      //PickPlace Action

	      /*
		if(sim){
		pick_place_goal.obj_centroid = Plan_Result.obj_centroid;
		}
		else{
		pick_place_goal.obj_centroid = Plan_Result.obj_centroid;
      
		}
	      */
	      push_vs_grasp::PickPlaceGoal pick_place_goal;
	      pick_place_goal.action_type = Plan_Result.action_type;
	      pick_place_goal.obj_centroid = Plan_Result.obj_centroid;
	      pick_place_goal.placement = Plan_Result.placement;
	      pick_place_goal.goal_reached = Plan_Result.goal_reached;
	      PickPlace_Action_client.sendGoal(pick_place_goal);
	      bool pick_place_result = PickPlace_Action_client.waitForResult();

	      push_vs_grasp::PickPlaceResult Pick_Place_Result = *PickPlace_Action_client.getResult();
	      int num_picks = Pick_Place_Result.numPickPlaces;
	      int num_pushes = Pick_Place_Result.numPushes;
	      actionlib::SimpleClientGoalState pick_place_state = PickPlace_Action_client.getState();
	      ROS_INFO("PickPlace Action finished: %s",pick_place_state.toString().c_str());    


	      // Time elapsed
	      ros::Time t_end = ros::Time::now();
	      time_elapsed = t_end - t_start;

	      if (Plan_Result.goal_reached || time_elapsed >= ros::Duration(timeout_secs)) {
		//Algorithms ==> 0 = pick/place only, push only, pick/place + push combination
		int algorithm = 0; 
		// Logging Data - appending to existing file
		myfile.open (concat.c_str(), std::fstream::app);
		if(Plan_Result.goal_reached){// Successful
		  myfile << "\n" << algorithm << "," << num_obj << "," << time_elapsed << "," << num_picks << "," << num_pushes << ", 1";
		}
		else{// Unsuccessful
		  myfile << "\n" << algorithm << "," << num_obj << "," << time_elapsed << "," << num_picks << "," << num_pushes << ", 0";
		}
		myfile.close();

		ROS_INFO("Goal Reached..");
		ROS_INFO("Time elapsed:");
		std::cout << time_elapsed;
		ROS_INFO("Number of Pick and Places:");
		std::cout << num_picks;
		ROS_INFO("Number of Pushes:");
		std::cout << num_pushes;
		break;
	      }

	    }
	}

    }
    
  return 0;
}
