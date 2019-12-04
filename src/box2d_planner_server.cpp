//General
#include <stdlib.h>
#include <iostream>
#include <time.h>       /* time */
#include <math.h>       /* sqrt */

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <push_vs_grasp/push_action.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>



#include "../Box2D/Testbed/Framework/Test.h"
#include "../Box2D/Testbed/Framework/Main.h"
#include "ApplyForce.h"

#include <boost/random.hpp>      
#include <boost/limits.hpp>
#include <vector> 

TestEntry g_testEntries[] =
{
  {"Apply Force", ApplyForce::Create},
  {NULL,NULL}
};

enum Color {red, blue};

class Box2DPlanner: public Test
{
 private:
  ros::NodeHandle nh_;
 // actionlib::SimpleActionServer<push_vs_grasp::PlanAction> as_;
  bool virgin = true;
  ApplyForce *test_derived;
  int seed = 5;

  float random_float(float min, float max) {

	return ((float)rand() / RAND_MAX) * (max - min) + min;

}
  
 
 public:

 void random_objects() {
   
   srand(seed++);


  float coll_radius = 0.08F;

  float min_x = (-0.7F + coll_radius);
  float max_x = 0;

  float min_y = -(0.75F-coll_radius);
  float max_y = 0.75F-coll_radius;

  int num_objs = 5;
  
  
  std::vector<geometry_msgs::PointStamped> objects;
  std::vector<double> radiuses;
  std::vector<std::string> colours;

  while (objects.size() < num_objs) {

      float x = random_float(min_x, max_x);
   
      float y = random_float(min_y, max_y);

      bool coll = false;
      
      for (geometry_msgs::PointStamped p : objects) {
        if ((p.point.x - x) * (p.point.x - x) + (p.point.y - y) * (p.point.y - y) < coll_radius * coll_radius) {
          coll = true;
          break;
        }
      }
 
      if (coll) {
        continue;        
      }

      geometry_msgs::PointStamped p ;
      p.point.x = x;//(-0.7F + coll_radius);
      p.point.y = y;

      objects.push_back(p);

      colours.push_back(random_float(0,1) > 0.5 ? "red" : "blue");

      radiuses.push_back(coll_radius);

  }

  geometry_msgs::PointStamped r1;
  r1.point.x = -0.7+0.2;
  r1.point.y = -(0.75-0.2);

  geometry_msgs::PointStamped r2;
  r2.point.x = -0.1;
  r2.point.y = (0.75-0.2);
  
  std::vector<double> goal_radiuses;
  goal_radiuses.push_back(0.2F);
  goal_radiuses.push_back(0.2F);

  test_derived->setup_objects(objects, radiuses, colours, r1, r2, goal_radiuses);

 }
 cv::Mat push(float start_x, float start_y, float angle, float dist, double& dist_out) {
   return test_derived->push_action(start_x, start_y, angle, dist, dist_out);
 }

  
  Box2DPlanner(ros::NodeHandle* nodehandle): virgin(true) {  // , nh_(*nodehandle), as_(nh_, "/box2d_planner", boost::bind(&Box2DPlanner::executeCB, this, _1),false) {
    //init_actionlib();
    setup_box2d(); //defined in Main.h
    test_derived = static_cast<ApplyForce*>(test);  
    random_objects();
  }
  
    ~Box2DPlanner() {
    g_debugDraw.Destroy();
    RenderGLDestroy();
    glfwTerminate();
  }



sensor_msgs::Image cv_to_ros(cv::Mat img) {

  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg; // >> message to be sent

  std_msgs::Header header; // empty header
  //header.seq = counter; // user defined counter
  header.stamp = ros::Time::now(); // time
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
  
  return img_msg;

}

bool do_action(push_vs_grasp::push_action::Request &req, push_vs_grasp::push_action::Response &res) {
  double dist;
  cv::Mat img = push(req.start_x, req.start_y, req.angle, req.dist, dist);

  res.done = dist == 0;
  res.reward = dist;

  res.next_state = cv_to_ros(img);

}


bool reset_action(push_vs_grasp::push_action::Request &req, push_vs_grasp::push_action::Response &res) {
  double dist;
  cv::Mat img = push(req.start_x, req.start_y, req.angle, req.dist, dist);

  res.done = dist == 0;
  res.reward = dist;

  res.next_state = cv_to_ros(img);

}

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "box2d_planner_server");
  ros::NodeHandle node_handle("~");

  Box2DPlanner B2DP(&node_handle);

  ros::ServiceServer ser = node_handle.advertiseService("push_action", &Box2DPlanner::do_action, &B2DP);

  ros::ServiceServer reset = node_handle.advertiseService("reset_action", &Box2DPlanner::reset_action, &B2DP);


  draw_stuff(true, true);
  double dist;
  B2DP.push(10, 20, 1.57, 30, dist);

  while (ros::ok()) {
            draw_stuff(true, true);
            // B2DP.push(-0.5, -0.5, 1.57, 1);              
  }
  
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
} 
