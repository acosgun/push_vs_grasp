//General
#include <stdlib.h>
#include <iostream>
#include <time.h>       /* time */
#include <math.h>       /* sqrt */

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

//moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


class Push_objects {
  private:

    ros::NodeHandle nh_;
    ros::Subscriber MarkerArray;
	boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    visualization_msgs::MarkerArray marker_array;

    void init_subs(){
      MarkerArray = nh_.subscribe("/scan_objects/object_markers", 1, &Push_objects::Push_Random, this); 
    }

    void init_objects(){
	  const std::string PLANNING_GROUP = "manipulator";
	  move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
      srand (time(NULL));
    }

    void init_collision_objects(){
    //###############################COLLISION OBJECTS INIT##########################################//
      //Create Collision Objects
      std::vector<moveit_msgs::CollisionObject> collision_objects;

      //Add table collision object
      moveit_msgs::CollisionObject Table_collision_object;
      Table_collision_object.header.frame_id = "base_link";//move_group.getPlanningFrame();
      Table_collision_object.id = "Table";
      //specify Table size
      shape_msgs::SolidPrimitive Table;
      Table.type = Table.BOX;
      Table.dimensions.resize(3);
      Table.dimensions[0] = 2;
      Table.dimensions[1] = 2;
      Table.dimensions[2] = 0.1;
      //specify Table position/orientation
      geometry_msgs::Pose Table_pose;
        Table_pose.orientation.w = 1.0;
      Table_pose.position.x = 0;
        Table_pose.position.y = 0;
        Table_pose.position.z = -0.055;
      //Create Table collision object
      Table_collision_object.primitives.push_back(Table);
      Table_collision_object.primitive_poses.push_back(Table_pose);
      Table_collision_object.operation = Table_collision_object.ADD;
      //Add Table collision object to collision objects
      collision_objects.push_back(Table_collision_object);

      //Add Wall collision object
      moveit_msgs::CollisionObject Wall_collision_object;
      Wall_collision_object.header.frame_id = "base_link";//move_group.getPlanningFrame();
      Wall_collision_object.id = "Wall";
      //specify Wall size
      shape_msgs::SolidPrimitive Wall;
      Wall.type = Wall.BOX;
      Wall.dimensions.resize(3);
      Wall.dimensions[0] = 0.5;
      Wall.dimensions[1] = 2;
      Wall.dimensions[2] = 1;
      //specify Wall position/orientation
      geometry_msgs::Pose Wall_pose;
        Wall_pose.orientation.w = 1.0;
      Wall_pose.position.x = 0.25 + 0.1; //Distance to edge of wall + gap between wall and base_link
        Wall_pose.position.y = 0;
        Wall_pose.position.z = 0.5;
      //Create Wall collision object
      Wall_collision_object.primitives.push_back(Wall);
      Wall_collision_object.primitive_poses.push_back(Wall_pose);
      Wall_collision_object.operation = Wall_collision_object.ADD;
      //Add Table collision object to collision objects
      collision_objects.push_back(Wall_collision_object);

      //Add vector of collision bjects to the "world"
      sleep(2);
      planning_scene_interface.addCollisionObjects(collision_objects);
    }


    void Push_Random (const   visualization_msgs::MarkerArrayConstPtr&  marker_array){
	  

	  move_group->setPlanningTime(5);
	  visualization_msgs::MarkerArray  marker_arrayXYZ = *marker_array; 
      move_group->setStartStateToCurrentState();
      geometry_msgs::Pose target_pose1 = move_group->getCurrentPose().pose;
        
      //Randomly choose a point from the array
      bool Point_success = false;
      int Marker_indx = rand() % (marker_arrayXYZ.markers.size() + 1);
      float X_pos = marker_arrayXYZ.markers[Marker_indx].pose.position.x;
      float Y_pos = marker_arrayXYZ.markers[Marker_indx].pose.position.y;
      float Z_pos = marker_arrayXYZ.markers[Marker_indx].pose.position.z;

      //make sure it's not the base position
      for(int i = 0; i<5; i++){ 
        if(sqrt(X_pos*X_pos+Y_pos*Y_pos+Z_pos*Z_pos) < 0.2){
          Marker_indx = rand() % (marker_arrayXYZ.markers.size()) + 1;
          X_pos = marker_arrayXYZ.markers[Marker_indx].pose.position.x;
          Y_pos = marker_arrayXYZ.markers[Marker_indx].pose.position.y;
          Z_pos = marker_arrayXYZ.markers[Marker_indx].pose.position.z;
          std::cout << "Robot_base!" << std::endl;
        }else{
          Point_success = true;
          break;
        } 
      }

      if(Point_success){
        target_pose1.position.x = X_pos;
        target_pose1.position.y = Y_pos - 0.1;
        target_pose1.position.z = Z_pos;

        ROS_INFO_NAMED("tutorial", "Current Pose After move x:%f y:%f z:%f ", target_pose1.position.x, target_pose1.position.y, target_pose1.position.z);

        move_group->setPoseTarget(target_pose1);

        bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success){
          //move to point
          move_group->move();

          target_pose1.position.y =  0.3;
          move_group->setPoseTarget(target_pose1);
          success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); 
          if(success){
            //Push to point
            move_group->move();
          }

          target_pose1.position.x = -0.4;
          target_pose1.position.y =  0.1;
          target_pose1.position.z =  0.4;
          move_group->setPoseTarget(target_pose1);

          success = false;
          for(int i = 0; i<3; i++){
            if(!success){
              success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); 
            }else{
              move_group->move();
              break;
            }
          }
        }	
      } else 		std::cout << "No Objects!" << std::endl;
	  sleep(1);
    }
    
  public:

    Push_objects(ros::NodeHandle* nodehandle):nh_(*nodehandle) {
      init_objects();
      init_subs();
      init_collision_objects();
    }
    
};


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "Push_Object_node");
  ros::NodeHandle node_handle("~");
  Push_objects PO(&node_handle);
  ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();
  return 0;
} 
