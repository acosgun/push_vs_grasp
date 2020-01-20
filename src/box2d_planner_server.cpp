//General
#include <stdlib.h>
#include <iostream>
#include <time.h>       /* time */
#include <math.h>       /* sqrt */

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

//Custom
#include <push_vs_grasp/PushAction.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>


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
  // actionlib::SimpleActionServer<push_vs_grasp::PlanAction> as_;
  actionlib::SimpleActionServer<push_vs_grasp::PushAction> push_as_;

  bool virgin = true;
  
  void init_actionlib(){
    push_as_.start();
    ROS_INFO("Box2DPlanner Server ON");
  }

  sensor_msgs::Image cv_to_ros(cv::Mat img)
  {
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;  // >> message to be sent

    std_msgs::Header header;  // empty header
    // header.seq = counter; // user defined counter
    header.stamp = ros::Time::now();  // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);

    img_bridge.toImageMsg(img_msg);  // from cv_bridge to sensor_msgs::Image

    return img_msg;
  }
  
  void executeCB (const actionlib::SimpleActionServer<push_vs_grasp::PushAction>::GoalConstPtr& goal)
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
    test_derived->push_action(goal->start_x, goal->start_y, goal->end_x, goal->end_y);

    cv::Mat img = test_derived->get_ocv_img_from_gl_img();
    
    push_vs_grasp::PushResult result_;
    result_.objects = test_derived->get_all_objects();
    result_.next_state = cv_to_ros(img); 
    result_.reward = test_derived->calc_heuristic();
    result_.done = result_.reward == 0;
    push_as_.setSucceeded(result_);
  }

 public:  
 Box2DPlanner(ros::NodeHandle* nodehandle): virgin(true), nh_(*nodehandle), push_as_(nh_, "/box2d_planner", boost::bind(&Box2DPlanner::executeCB, this, _1),false) {
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
