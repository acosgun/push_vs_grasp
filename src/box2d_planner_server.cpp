#include "box2d_planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "box2d_planner_server");
  ros::NodeHandle node_handle("~");
  Box2DPlanner B2DP(&node_handle);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
} 
