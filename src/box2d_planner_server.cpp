//General
#include <stdlib.h>
#include <iostream>
#include <time.h>       /* time */
#include <math.h>       /* sqrt */

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

//Custom
#include <push_vs_grasp/PlanAction.h>

//BOX2D
#include "../Box2D/Testbed/Framework/Test.h"
#include "../Box2D/Testbed/Framework/Main.h"
#include "ApplyForce.h"

TestEntry g_testEntries[] =
{
  {"Apply Force", ApplyForce::Create},
  {NULL,NULL}
};

class Box2DPlanner: public Test
{
 private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<push_vs_grasp::PlanAction> as_;
  bool virgin = true;
  bool enable_draw = false;
  
  void init_actionlib(){
    as_.start();
    ROS_INFO("Box2DPlanner Server ON");
  }
  
  void executeCB (const actionlib::SimpleActionServer<push_vs_grasp::PlanAction>::GoalConstPtr& goal)
  {
    ROS_INFO("PlanAction Server: executeCB");
    
    if (virgin == true) {	
	setup_box2d(); //defined in Main.h
	virgin = false;
      }

    ApplyForce *test_derived = static_cast<ApplyForce*>(test);
    test_derived->destroy_all_objects();
    test_derived->setup_objects(goal->centroids, goal->radiuses, goal->colors, goal->red_goal, goal->blue_goal, goal->goal_radiuses);
    geometry_msgs::PointStamped obj_centroid;
    geometry_msgs::PointStamped placement;
    bool goal_reached;
    int action_type;
    test_derived->plan(obj_centroid, placement, goal_reached, action_type, enable_draw, goal->algo);

    
    push_vs_grasp::PlanResult result_;
    result_.obj_centroid = obj_centroid;
    result_.placement = placement;
    result_.goal_reached = goal_reached;
    result_.action_type = action_type;
    as_.setSucceeded(result_);
  }

 public:  
 Box2DPlanner(ros::NodeHandle* nodehandle): virgin(true), nh_(*nodehandle), as_(nh_, "/box2d_planner", boost::bind(&Box2DPlanner::executeCB, this, _1),false) {
   nh_.getParam("enable_draw", enable_draw);
   init_actionlib();
  }
  
    ~Box2DPlanner() {
    g_debugDraw.Destroy();
    RenderGLDestroy();
    glfwTerminate();
  }

};


int main(int argc, char** argv) {
  ros::init(argc, argv, "box2d_planner_server");
  ros::NodeHandle node_handle("~");
  Box2DPlanner B2DP(&node_handle);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
} 
