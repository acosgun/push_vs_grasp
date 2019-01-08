#include "moveit_push.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "push_server");
  ros::NodeHandle node_handle("~");
  Push_objects PO(&node_handle);
  ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();
  return 0;
} 
