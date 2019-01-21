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
  
  void init_actionlib(){
    as_.start();
    ROS_INFO("Box2DPlanner Server ON");
  }
  
  void executeCB (const actionlib::SimpleActionServer<push_vs_grasp::PlanAction>::GoalConstPtr& goal)
  {    
    if (virgin == true) {	
	setup_box2d(); //defined in Main.h
	virgin = false;
      }
 
    ApplyForce *test_derived = static_cast<ApplyForce*>(test);
    test_derived->destroy_all_objects();
    test_derived->setup_table();
    test_derived->setup_objects(goal->all_centroids);

    while(true) {
      draw_stuff(); //defined in Main.h
      sleep(0.25);
      break;
    }
    
    push_vs_grasp::PlanResult result_;
    as_.setSucceeded(result_);
  }

 public:  
 Box2DPlanner(ros::NodeHandle* nodehandle): virgin(true), nh_(*nodehandle), as_(nh_, "/box2d_planner", boost::bind(&Box2DPlanner::executeCB, this, _1),false) {
    init_actionlib();
  }
  
    ~Box2DPlanner() {
    g_debugDraw.Destroy();
    RenderGLDestroy();
    glfwTerminate();
  }

};
