// #include <stdlib.h>
// #include <string.h>
// #include <iostream>
// #include <time.h>       /* time */
// #include <boost/thread.hpp>
// //ROS
// #include <ros/ros.h>
// #include <actionlib/client/simple_action_client.h>
// #include <actionlib/client/terminal_state.h>

// #include <kinect_segmentation/ScanObjectsAction.h>
// #include <kinect_segmentation/ScanObjectsGoal.h>

// #include <push_vs_grasp/PlanAction.h>
// #include <push_vs_grasp/PlanGoal.h>

// #include <push_vs_grasp/PickPlaceAction.h>
// #include <push_vs_grasp/PickPlaceGoal.h>


// // Global Variables
// float x_min =-0.2, x_max =0.2, y_min =-0.3, y_max =-0.55;
// float red_radius =0.10, blue_radius =0.10;
// float object_radius = 0.03;
// bool simulation = true, perfect_perception = true;

// void spinThread()
// {
//   ros::spin();
// }

int main (int argc, char **argv) {}
// {
//   ros::init(argc, argv, "push_client");
//   srand (time(NULL));

//   actionlib::SimpleActionClient<kinect_segmentation::ScanObjectsAction> ScanObjects_Action_client("scan_objects");
//   actionlib::SimpleActionClient<push_vs_grasp::PlanAction> Plan_Action_client("box2d_planner");
//   actionlib::SimpleActionClient<push_vs_grasp::PickPlaceAction> PickPlace_Action_client("pick_place");

//   boost::thread spin_thread(&spinThread);

//   ROS_INFO("Waiting for action Client to startup.");
//   ScanObjects_Action_client.waitForServer(); //will wait for infinite time
//   ROS_INFO("ScanObjects_Action_client started");
//   PickPlace_Action_client.waitForServer(); //will wait for infinite time
//   ROS_INFO("PickPlace_Action_client started");
//   Plan_Action_client.waitForServer(); //will wait for infinite time
//   ROS_INFO("Plan_Action_client started");

//   bool plan_success = false;


//   // Randomly generate goal regions and objects
//   // Generation of goal regions
//   geometry_msgs::PointStamped redGoal_base_link;
//   geometry_msgs::PointStamped blueGoal_base_link;


//   redGoal_base_link.header.frame_id = "base_link";
//   blueGoal_base_link.header.frame_id = "base_link";


//   double x1, x2, y1, y2;
  
//   std::random_device rd;
//   std::mt19937 gen(rd());
//   // Distribution encompasses region projected by projector
//   // Vertical Limits
//   std::uniform_real_distribution<> dist1(y_min, y_max); // Need to make table bigger in box2d and gazebo //REAL: (-0.17,-0.85);
//   // Horizontal Limits
//   std::uniform_real_distribution<> dist2(x_min, x_max); // REAL: (-0.465, 0.75)
 
 
//   while(true){


//     // 


//     // Random generation of goal regions
//     bool goalCheck = 1;

//     while(goalCheck){
//       x1 = dist1(gen);
//       x2 = dist1(gen);
//       y1 = dist2(gen);
//       y2 = dist2(gen);

//       if(((x2 > (x1+red_radius)) || (x2 < (x1-red_radius))) && ((y2 > (y1+red_radius)) || (y2 < (y1-red_radius)))){
//         goalCheck = 0;
//       }
//     }

//     redGoal_base_link.point.x = x1;
//     blueGoal_base_link.point.x = x2;
//     redGoal_base_link.point.y = y1;
//     blueGoal_base_link.point.y = y2;
//     redGoal_base_link.point.z = 0;
//     blueGoal_base_link.point.z = 0;


//     // If perfect perception, read the positions of the objects directly rather than through perception

    
//     /*
//     // Random generation of objects
//     // Randomly choosing number of objects to spawn (between 1 and 10)
//     bool objectCheck = true;
//     float x1, y1;
//     int numObjects =  (rand() % 10) + 1; // randomly generating number between 1 and 10
//     std::vector<float> x_vals;
//     std::vector<float> y_vals;
//     std::vector<std::string> colors;

//     // Determining colours of objects
//     for(int i = 0; i < numObjects; i++){
//       // Red = 0, Blue = 1
//       if(rand()%2){
//         colors.push_back("red");
//       }
//       else{
//         colors.push_back("blue");
//       }
//     }

//     // Determining positions of objects
//     std::vector<geometry_msgs::PointStamped> object_centroids;
//     geometry_msgs::PointStamped temp;

//     temp.point.x = dist1(gen);
//     temp.point.y = dist2(gen);

//     object_centroids.push_back(temp);

//     while(objectCheck && object_centroids.size() < numObjects){ // Positions of objects
//       x1 = dist1(gen);
//       y1 = dist2(gen);

      
//       // Checking objects aren't overlapping
//       for(int i = 0; i < object_centroids.size(); i++){
//         // Ensuring collision free placement of objects
//         if(x1 > object_centroids[i].point.x + 2*object_radius || x1 < object_centroids[i].point.x - 2*object_radius || y1 < object_centroids[i].point.y - 2*object_radius || y1 > object_centroids[i].point.y + 2*object_radius){
//           if(i == object_centroids.size() - 1){ // If no collisions with other objects
            
//             temp.point.x = x1;
//             temp.point.y = y1;
//             object_centroids.push_back(temp);  
//           }
//         }
//         else{
//           break;
//         }

//         if(i >= numObjects - 1){ // if the position of all the objects have been successfully determined
//           objectCheck = false;
//           break;
//         }
//       }

//     }
//     */
    
//     ros::Time t_start = ros::Time::now();

//     while(true)
//     { 

      

//       //Scan Action
//       kinect_segmentation::ScanObjectsGoal ScanObjects_goal;
//       ScanObjects_goal.goal_reached = plan_success;
//       ScanObjects_goal.red_goal = redGoal_base_link;
//       ScanObjects_goal.blue_goal = blueGoal_base_link;
//       ScanObjects_goal.goal_radiuses.push_back(red_radius);
//       ScanObjects_goal.goal_radiuses.push_back(blue_radius);


//       ScanObjects_Action_client.sendGoal(ScanObjects_goal);
//       ScanObjects_Action_client.waitForResult();
//       kinect_segmentation::ScanObjectsResult Scan_Result = *ScanObjects_Action_client.getResult();
//       actionlib::SimpleClientGoalState scan_state = ScanObjects_Action_client.getState();
//       ROS_INFO("Scan Action finished: %s",scan_state.toString().c_str());

//       if (scan_state == actionlib::SimpleClientGoalState::SUCCEEDED){
//         if (Scan_Result.centroids.empty()){
//           ROS_INFO("Table is EMPTY!");
//           return 0;
//         }
//       }
      
//       //Plan Action

//       /*
//       if(simulation){
//         planner_goal.centroids = Scan_Result.centroids;
//         planner_goal.radiuses = Scan_Result.radiuses;
//         planner_goal.colors = colors;
//       }
//       else{
//         planner_goal.centroids = Scan_Result.centroids;
//         planner_goal.radiuses = Scan_Result.radiuses;
//         planner_goal.colors = Scan_Result.colors;
      
//       }


//       */

//       push_vs_grasp::PlanGoal planner_goal;
//       planner_goal.centroids = Scan_Result.centroids;
//       planner_goal.radiuses = Scan_Result.radiuses;
//       planner_goal.goal_radiuses = ScanObjects_goal.goal_radiuses;
//       planner_goal.colors = Scan_Result.colors;
//       planner_goal.red_goal = ScanObjects_goal.red_goal;
//       planner_goal.blue_goal = ScanObjects_goal.blue_goal;
//       Plan_Action_client.sendGoal(planner_goal);
//       bool plan_result = Plan_Action_client.waitForResult();
//       push_vs_grasp::PlanResult Plan_Result = *Plan_Action_client.getResult();
//       plan_success = Plan_Result.goal_reached;
//       actionlib::SimpleClientGoalState plan_state = Plan_Action_client.getState();
//       ROS_INFO("Plan Action finished: %s",plan_state.toString().c_str());

//       if (Plan_Result.goal_reached) {
//         ROS_INFO("Goal Reached..");
//         ros::Time t_end = ros::Time::now();
//         ROS_INFO("Time elapsed:");
//         std::cout << t_end - t_start;
//         break;
//       }
      
//       //PickPlace Action

//       /*
//       if(simulation){
//         pick_place_goal.obj_centroid = Plan_Result.obj_centroid;
//       }
//       else{
//         pick_place_goal.obj_centroid = Plan_Result.obj_centroid;
      
//       }
//       */
//       push_vs_grasp::PickPlaceGoal pick_place_goal;
//       pick_place_goal.action_type = Plan_Result.action_type;
//       pick_place_goal.obj_centroid = Plan_Result.obj_centroid;
//       pick_place_goal.placement = Plan_Result.placement;
//       PickPlace_Action_client.sendGoal(pick_place_goal);
//       bool pick_place_result = PickPlace_Action_client.waitForResult();
//       actionlib::SimpleClientGoalState pick_place_state = PickPlace_Action_client.getState();
//       ROS_INFO("PickPlace Action finished: %s",pick_place_state.toString().c_str());      
//     }

//   }
    
//   return 0;
// }
