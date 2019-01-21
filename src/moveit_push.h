//General
#include <stdlib.h>
#include <iostream>
#include <time.h>       /* time */
#include <math.h>       /* sqrt */

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <actionlib/server/simple_action_server.h>


//moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

//Custom
#include <push_vs_grasp/MoveItPushAction.h>
#define MIN_Z_DEPTH 0.08
#define PUSH_OFFSET 0.15

class Push_objects {
  private:

    ros::NodeHandle nh_;
	  boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    actionlib::SimpleActionServer<push_vs_grasp::MoveItPushAction> as_;

	  std::vector<moveit_msgs::CollisionObject> Scene_collision_objects;
    std::vector<moveit_msgs::CollisionObject> Item_collision_objects;
    geometry_msgs::Pose target_pose;
	  geometry_msgs::Pose goal_pose;
    geometry_msgs::Pose home_pose;

    void init_actionlib(){
        as_.start();
        ROS_INFO("Push Server ON");
    }  

    void init_objects(){
	    const std::string PLANNING_GROUP = "manipulator";
	    move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
      srand (time(NULL));

      home_pose.position.x = -0.4;
      home_pose.position.y = 0.1;
      home_pose.position.z = 0.4;
      home_pose.orientation.w = 0;
      home_pose.orientation.x = 0;
      home_pose.orientation.y = 1;
      home_pose.orientation.z = 0;
    }

    void init_collision_objects(){
    //###############################COLLISION OBJECTS INIT##########################################//
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
      Scene_collision_objects.push_back(Table_collision_object);

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
      Wall_pose.position.x = 0.25 + 0.2; //Distance to edge of wall + gap between wall and base_link
	    Wall_pose.position.y = 0;
      Wall_pose.position.z = 0.5;
      //Create Wall collision object
      Wall_collision_object.primitives.push_back(Wall);
      Wall_collision_object.primitive_poses.push_back(Wall_pose);
      Wall_collision_object.operation = Wall_collision_object.ADD;
      //Add Table collision object to collision objects
      Scene_collision_objects.push_back(Wall_collision_object);
      //Add vector of collision objects to the "world"
      sleep(2);
      planning_scene_interface.addCollisionObjects(Scene_collision_objects);
    }
  
    void add_collision_Items(std::vector<geometry_msgs::PointStamped> Centroids){
      //Add Items as collision objects
      moveit_msgs::CollisionObject Item_collision_object;
      Item_collision_object.header.frame_id = "base_link";
      //specify Cylinder size
      shape_msgs::SolidPrimitive Item;
      Item.type = Item.CYLINDER;
      Item.dimensions.resize(2);
      Item.dimensions[0] = 0.12;
      Item.dimensions[1] = 0.035;
      //specify Table position/orientation
      geometry_msgs::Pose Item_pose;

      for(int i = 0; i < static_cast<int>(Centroids.size()); i++){
        Item_collision_object.id = i;
        Item_pose.orientation.w = 1.0;
        Item_pose.position.x = Centroids[i].point.x;
        Item_pose.position.y = Centroids[i].point.y;
        Item_pose.position.z = Centroids[i].point.z;

        Item_collision_object.primitives.push_back(Item);
        Item_collision_object.primitive_poses.push_back(Item_pose);
        Item_collision_object.operation = Item_collision_object.ADD;
        //Add collision object to collision objects
        Item_collision_objects.push_back(Item_collision_object);
      }
      //Add vector of collision objects to the "world"
      sleep(2);
      planning_scene_interface.addCollisionObjects(Item_collision_objects);
    }

    void Remove_collision_Items(){
      std::vector<std::string> object_ids;
      for(int i = 0; i < static_cast<int>(Item_collision_objects.size()); i++){
        object_ids.push_back(Item_collision_objects[i].id);
      }

      planning_scene_interface.removeCollisionObjects(object_ids);
    }


    bool PlanCartesian_ToPoint(){
      std::cout << "PlanCartesian_ToPoint NOW!" << std::endl;
      move_group->setStartStateToCurrentState();
      geometry_msgs::Pose WayPoint = move_group->getCurrentPose().pose;
      std::vector<geometry_msgs::Pose> WayPoints;
      float Y_offset = 0;
      if (goal_pose.position.y > target_pose.position.y){
        Y_offset = -PUSH_OFFSET;
      }else Y_offset = PUSH_OFFSET;

      //plan to above object
        WayPoint.position.y = target_pose.position.y + Y_offset;
        WayPoints.push_back(WayPoint);  // down

        WayPoint.position.x = target_pose.position.x;
        WayPoints.push_back(WayPoint);  // down

        WayPoint.position.z = target_pose.position.z + 0.1;
        WayPoints.push_back(WayPoint);  // down

      //plan to next to object
        WayPoint.position.z = MIN_Z_DEPTH;
        WayPoints.push_back(WayPoint);  // right

      //create cartesian path trajectory msg
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.05;
        double fraction = move_group->computeCartesianPath(WayPoints, eef_step, jump_threshold, trajectory, true);
  /* 
      //Create a RobotTrajectory object  - method to use if errors with empty velocity vectors 
      robot_trajectory::RobotTrajectory rt(move_group->getCurrentState()->getRobotModel(), "manipulator");
      rt.setRobotTrajectoryMsg(*move_group->getCurrentState(), trajectory);

      //Create a IterativeParabolicTimeParameterization object
        trajectory_processing::IterativeParabolicTimeParameterization iptp;

      // Get RobotTrajectory_msg from RobotTrajectory
        rt.getRobotTrajectoryMsg(trajectory);

      bool success = iptp.computeTimeStamps(rt);;
  */
      std::cout << fraction << std::endl;
      bool success = false;
        // Plan the trajectory
      if (fraction == 1){
          my_plan.trajectory_ = trajectory;
        success = true;
      }
      return success;
    }

    bool PlanCartesian_Push(){
      std::cout << "PlanCartesian_Push NOW!" << std::endl;
      geometry_msgs::Pose StartPoint = move_group->getCurrentPose().pose;

      geometry_msgs::Pose WayPoint = StartPoint;
      std::vector<geometry_msgs::Pose> WayPoints;

      float Xdiff = (goal_pose.position.x - StartPoint.position.x);
      float Ydiff = (goal_pose.position.y - StartPoint.position.y);
      float GradM = (Ydiff/Xdiff);
      float Yoffset = goal_pose.position.y - GradM * goal_pose.position.x;
      int steps = 1; 
      float Zangle = - tanh(1/GradM);

      tf::Quaternion q_rot, q_orig,q_new;
      q_rot.setRPY(0,0,Zangle);

      tf::quaternionMsgToTF(WayPoint.orientation , q_orig);
      q_new = q_rot*q_orig;  // Calculate the new orientation
      q_new.normalize();
      tf::quaternionTFToMsg(q_new, WayPoint.orientation);

      WayPoints.push_back(WayPoint); 
      for(int i = 0; i<steps; i++){
        WayPoint.position.x = StartPoint.position.x + (Xdiff/(steps+1));
        WayPoint.position.y = (WayPoint.position.x * GradM) + Yoffset;
        WayPoints.push_back(WayPoint); 
      }

      WayPoint.position.x = goal_pose.position.x;
      WayPoint.position.y = goal_pose.position.y;
      WayPoint.position.z = MIN_Z_DEPTH;

      WayPoints.push_back(goal_pose);  // down

      //create cartesian path trajectory msg
      moveit_msgs::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.05;
      double fraction = move_group->computeCartesianPath(WayPoints, eef_step, jump_threshold, trajectory, true);
      bool success = false;
      std::cout << fraction << std::endl;

      if (fraction == 1){
          my_plan.trajectory_ = trajectory;
        success = true;
      }
      return success;

    }

    bool PlanCartesian_ToHome(){
      std::cout << "PlanCartesian_ToHome NOW!" << std::endl;

      geometry_msgs::Pose StartPoint = move_group->getCurrentPose().pose;

      geometry_msgs::Pose WayPoint = StartPoint;
      std::vector<geometry_msgs::Pose> WayPoints;

      WayPoint.position.y = 0.1;
      WayPoint.position.z = 0.25;
      WayPoints.push_back(WayPoint);

      WayPoints.push_back(home_pose);  // down

      //create cartesian path trajectory msg
      moveit_msgs::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.05;
      bool success = false;
      double fraction = 0.0;
      for(int i = 0; i<5;i++){
        fraction = move_group->computeCartesianPath(WayPoints, eef_step, jump_threshold, trajectory, true);
        std::cout <<"home: "<< fraction <<" attempt: "<<i<< std::endl;

        if (fraction == 1){
          my_plan.trajectory_ = trajectory;
          success = true;
          break;
        }

      }
      return success;
    }

    void Push_Object (const actionlib::SimpleActionServer<push_vs_grasp::MoveItPushAction>::GoalConstPtr& goal){
    

      geometry_msgs::PointStamped goalXYZ = goal->obj_centroid; 
      std::vector<geometry_msgs::PointStamped> Centroids = goal->all_centroids;
      push_vs_grasp::MoveItPushResult result_;

      move_group->setStartStateToCurrentState();
      move_group->setGoalJointTolerance(0.1);
      target_pose = move_group->getCurrentPose().pose;
      move_group->setPlanningTime(5);
      move_group->setMaxVelocityScalingFactor(0.1);

      //add_collision_Items(Centroids);

      std::cout << "w: " <<target_pose.orientation.w << " x: " <<target_pose.orientation.x << " y: " <<target_pose.orientation.y<< " z: " <<target_pose.orientation.z<< std::endl;

      //create goal pose
      goal_pose = target_pose;
      goal_pose.position.x = -0.4;
      goal_pose.position.y = 0.5;
      goal_pose.position.z = 0.1;

      target_pose.position.x = goalXYZ.point.x;
      target_pose.position.y = goalXYZ.point.y;
      target_pose.position.z = goalXYZ.point.z;
      bool success;
/*
      //Randomly choose a point from the array
      int Marker_indx = rand() % (static_cast<int>(Centroids.size()) + 1);
      target_pose.position.x = Centroids[Marker_indx].position.x;
      target_pose.position.y = Centroids[Marker_indx].position.y;
      target_pose.position.z = Centroids[Marker_indx].position.z;
*/

      success = PlanCartesian_ToPoint();
      //Remove_collision_Items();
      result_.result = success;
      if(success){
        //move to point
        move_group->execute(my_plan);
        //push to goal
        sleep(1);
        success = PlanCartesian_Push();
        result_.result = success;
        if(success){
          //Push to point
          move_group->execute(my_plan);
          sleep(1);
        }
        success = PlanCartesian_ToHome();
        result_.result = success;
        if(success){
          //go to home
          std::cout <<"go home" << std::endl;
          move_group->execute(my_plan);
          as_.setSucceeded(result_);
        }
        sleep(1);
      }
    }
    
  public:

    Push_objects(ros::NodeHandle* nodehandle): nh_(*nodehandle), as_(nh_, "/Pushing", boost::bind(&Push_objects::Push_Object, this, _1),false) {
      init_objects();
      init_actionlib();
      init_collision_objects();
    }
    
};
