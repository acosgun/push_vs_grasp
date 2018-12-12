#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>
#define PI 3.14

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


  geometry_msgs::Pose cur_pose = move_group.getCurrentPose().pose;  

  geometry_msgs::Pose target_pose1 = cur_pose;
  target_pose1.position.x = -0.5;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.0;

  geometry_msgs::PoseStamped pose = move_group.getCurrentPose();
  
  std::cout <<" x:" << move_group.getCurrentPose().pose.position.x <<" y:" << move_group.getCurrentPose().pose.position.y <<" z:" << move_group.getCurrentPose().pose.position.z << std::endl;

  std::vector<double> tolerance_pose(3);
  tolerance_pose[0] = 1;
  tolerance_pose[1] = 1;
  tolerance_pose[2] = 0.2;

  std::vector<double> tolerance_angle(3, PI/2);
  
  moveit_msgs::Constraints test_constraints = kinematic_constraints::constructGoalConstraints("ee_link", pose, tolerance_pose, tolerance_angle);

  // Now, set it as the path constraint for the group.
  move_group.setPathConstraints(test_constraints);

  move_group.setPoseTarget(target_pose1);
  move_group.setStartStateToCurrentState();

  move_group.setPlanningTime(20.0);
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //*/
  if(success){
    move_group.move();
  }

    return 0;
}