#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  move_group.setStartStateToCurrentState();

  geometry_msgs::Pose cur_pose = move_group.getCurrentPose().pose;  

  double dx = -0.1;
  double dy = 0.0;
  double dz = 0.0;
  
  geometry_msgs::Pose target_pose1 = move_group.getCurrentPose().pose;  
  target_pose1.position.x = cur_pose.position.x + dx;
  target_pose1.position.y = cur_pose.position.y + dy;
  target_pose1.position.z = cur_pose.position.z + dz;

  move_group.setPoseTarget(target_pose1);
  move_group.move();
  
}
