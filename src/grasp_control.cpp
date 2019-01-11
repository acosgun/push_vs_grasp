
// ROS
#include <ros/ros.h>
#include <iostream>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <std_msgs/Float32.h>

class Gripper_objects {
  private:
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output Grip;
    ros::Publisher pub_to_gripper;
    ros::Subscriber gripper_info;

    void init_subs(){
    gripper_info = nh_.subscribe("Robotiq2FGripperRobotInput", 1, &Gripper_objects::Gripper_move, this); 
    }

    void init_pubs(){
        object_markers_pub_ = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output> ("Robotiq2FGripperRobotOutput", 1);
    }

    void Gripper_init(){
        Grip.rACT = 0;
        Grip.rPR = 0;
        Grip.rGTO = 0;
        Grip.rSP  = 0;
        Grip.rFR = 0;
        Grip.rATR =0;
        pub_to_gripper.publish(Grip);
        sleep(2);

        Grip.rACT = 1;
        Grip.rPR = 0;
        Grip.rGTO = 1;
        Grip.rSP  = 255;
        Grip.rFR = 150;
        Grip.rATR =0;
        pub_to_gripper.publish(Grip);
        sleep(2);
    }

    void Gripper_close(){
        Grip.rPR = 255;
        pub_to_gripper.publish(Grip);
        sleep(2);

    }

    void Gripper_open(){
        Grip.rPR = 0;
        pub_to_gripper.publish(Grip);
        sleep(2);
    }

    void Gripper_move(){

        for(int i=0;i<10;i++){
            Gripper_close();
            Gripper_open();
        }
    }

    public:

    Gripper_objects(ros::NodeHandle* nodehandle): nh_(*nodehandle) {
      init_subs();
      init_pubs();
      Gripper_init();
    }

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_test");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  Gripper_objects Gripper_object(&nh);
  spinner.start();
  return 0;
}