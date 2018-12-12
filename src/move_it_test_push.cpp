
// ROS
#include <ros/ros.h>
#include <iostream>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <visualization_msgs/MarkerArray.h>


void Move_ur5(const   visualization_msgs::MarkerArrayConstPtr&  marker_array){
    std::cout << "MOVE!" << std::endl;

		visualization_msgs::MarkerArray  marker_arrayXYZ = *marker_array; 
	/*
		for(int i = 0; i <marker_arrayXYZ.markers.size(); i++){
        	std::cout <<"x:"<<marker_arrayXYZ.markers[i].pose.position.x<<" y:"<<marker_arrayXYZ.markers[i].pose.position.y<<" z:"<<marker_arrayXYZ.markers[i].pose.position.z << std::endl;
		}
	*/
///*
		static const std::string PLANNING_GROUP = "manipulator";
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

		// We will use the :planning_scene_interface:`PlanningSceneInterface` ee_link
		// class to add and remove collision objects in our "virtual world" scene
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
 		moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        //
        // Next get the current set of joint values for the group.
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
        std::cout << joint_group_positions[0] <<" "<<joint_group_positions[1] <<" "<<joint_group_positions[2] <<" "<<joint_group_positions[3] <<" "<<joint_group_positions[4] <<" "<<joint_group_positions[5] << std::endl;

		//add rviz tools
		namespace rvt = rviz_visual_tools;
		moveit_visual_tools::MoveItVisualTools visual_tools("ur5");
		visual_tools.deleteAllMarkers();

		visual_tools.loadRemoteControl();

		Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
 		text_pose.translation().z() = 1.75;
		visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

		//add test collision box
		moveit_msgs::CollisionObject collision_object;
		collision_object.header.frame_id = move_group.getPlanningFrame();
		collision_object.id = "box1";
		//specify box size
		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[0] = 2;
		primitive.dimensions[1] = 2;
		primitive.dimensions[2] = 0.1;
		//specify box position/orientation
		geometry_msgs::Pose box_pose;
  		box_pose.orientation.w = 1.0;
		box_pose.position.x = 0;
  		box_pose.position.y = 0;
  		box_pose.position.z = -0.05;

		//add box to collision object
		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(box_pose);
		collision_object.operation = collision_object.ADD;
		//add box collision object to vector of Collision Objects
		std::vector<moveit_msgs::CollisionObject> collision_objects;
		collision_objects.push_back(collision_object);
		//add vector of collision bjects to the "world"
		sleep(2);
		planning_scene_interface.addCollisionObjects(collision_objects);

		move_group.setStartStateToCurrentState();

		geometry_msgs::Pose target_pose1 = move_group.getCurrentPose().pose;
		//target_pose1.orientation.x = 0;
        
		target_pose1.position.x = marker_arrayXYZ.markers[0].pose.position.x;
		target_pose1.position.y = marker_arrayXYZ.markers[0].pose.position.y;
		target_pose1.position.z = marker_arrayXYZ.markers[0].pose.position.z;
		

		ROS_INFO_NAMED("tutorial", "Current Pose After move x:%f y:%f z:%f ", target_pose1.position.x, target_pose1.position.y, target_pose1.position.z);

	 	move_group.setPoseTarget(target_pose1);

        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if(success){
            move_group.move();

            //back to home
            target_pose1.position.z = 0.5;
            target_pose1.position.y = 0.1;
            target_pose1.position.x = -0.5;

            move_group.setPoseTarget(target_pose1);

            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);   
            if(success){
                move_group.move();
            }
        }
//*/
}


int main(int argc, char** argv)
{
	std::cout << "INIT" << std::endl;
	ros::init(argc, argv, "move_robot");
	ros::NodeHandle node_handle;
	ros::Subscriber sub = node_handle.subscribe ("/segment_tabletop_node/object_markers", 1, &Move_ur5);
   	ros::AsyncSpinner spinner(2);

	spinner.start();
	  ros::waitForShutdown();
  return 0; 

}