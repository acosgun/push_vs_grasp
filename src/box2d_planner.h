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
  {"Apply Force", ApplyForce::Create}
};

class Box2DPlanner: public Test
{
 private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<push_vs_grasp::PlanAction> as_;
  ApplyForce *test;
  
  void init_actionlib(){
    as_.start();
    ROS_INFO("Box2DPlanner Server ON");
  }
  void executeCB (const actionlib::SimpleActionServer<push_vs_grasp::PlanAction>::GoalConstPtr& goal)
  {

    double my_num = test->reply();
    std::cout<<my_num<<std::endl;

    //TODO: Draw Objects here!
    
    /*
    while (!glfwWindowShouldClose(mainWindow))
      {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	sSimulate();
	glfwSwapBuffers(mainWindow);	
	glfwPollEvents();
      }
    */
  }

 public:  
    Box2DPlanner(ros::NodeHandle* nodehandle): nh_(*nodehandle), as_(nh_, "/box2d_planner", boost::bind(&Box2DPlanner::executeCB, this, _1),false)
    {
      setup_box2d();
      init_actionlib();       
      
      Test* test_base;    
      test_base = g_testEntries->createFcn();
      test = static_cast<ApplyForce*>(test_base);

      //delete test_base;      
      //test->setup_scene();      
      //double my_num = test->reply();
      //std::cout<<my_num<<std::endl;        
    }
  
    ~Box2DPlanner() {
    g_debugDraw.Destroy();
    RenderGLDestroy();
    glfwTerminate();
  }

};
