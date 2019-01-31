#! /usr/bin/env python
import sys
#ROS
import roslib
roslib.load_manifest('push_vs_grasp')
import copy
import rospy
import actionlib
from moveit_python import *
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import *
import time
#CUSTOM
from push_vs_grasp.msg import PickPlaceAction

#GLOBAL VARIABLES
gripperOffset = 0.25
gripperOffset_sim = 0.32
class PickPlaceServer:
  def __init__(self):
    #Gazebo Simluation

    self.sim = rospy.myargv(argv=sys.argv)[1]
    
    self.init_moveit()

    self.init_gazebo()
    self.init_real()

    self.obj_name ="n/a"
    self.obj_state = 'n/a'


    self.home_pose = geometry_msgs.msg.Pose()
    self.Goal_pose = geometry_msgs.msg.Pose()
    self.Target_pose = geometry_msgs.msg.Pose()

    self.server = actionlib.SimpleActionServer('pick_place', PickPlaceAction, self.executeCB, False)

    self.init_home_pose()

    self.go_home()
    self.server.start()    
    rospy.loginfo("Pick Place Server ON")

  def init_moveit(self):

    robot = moveit_commander.RobotCommander()

    if self.sim == "true":
      self.sim = True
    print self.sim
    scene = moveit_commander.PlanningSceneInterface("base_link")
    time.sleep(2)
    # Create table obstacle
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    print p.header.frame_id
    p.pose.position.x = -0.5
    p.pose.position.y = 0
    p.pose.position.z = -0.03
    scene.add_box("table", p, (2, 2, 0.06))
    time.sleep(2)

    group_name = "manipulator"
    self.group_name = group_name
    group = moveit_commander.MoveGroupCommander(self.group_name)
    planning_frame = group.get_planning_frame()
    #print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    #print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""
    self.group = group

  def init_gazebo(self):

    if(self.sim):
      #time.sleep(5)
      rospy.wait_for_service("gazebo/get_model_state")
      rospy.wait_for_service("gazebo/set_model_state")
      self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
      self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

  def init_real(self):
    if not (self.sim):
      self.Grip = Robotiq2FGripper_robot_output()
      self.pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', Robotiq2FGripper_robot_output, queue_size=10)
      self.init_gripper()


  def vanish_gazebo_object(self, obj_name):
    state = ModelState()
    state.model_name = obj_name
    pose = Pose()
    pose.position.x = 10.0
    pose.position.y = 10.0
    pose.position.z = 10.0
    state.pose = pose
    self.set_model_state(state)

  def get_closest_object(self, closest_obj):
    max_num_objs = 5
    model_name = "unit_cylinder"
    import sys
    min_dist = sys.float_info.max
    final_name = 'n/a'
    final_response = 'n/a'  
    for i in range(max_num_objs):          
      cur_obj_name = model_name + str(i)
      response = self.get_model_state(cur_obj_name,  "robot::base_link")
      if response.success:
        import numpy as np
        closest_obj_pos = np.array([closest_obj.x, closest_obj.y, closest_obj.z])
        response_obj_pos =np.array([response.pose.position.x, response.pose.position.y, response.pose.position.z])       
        dist = np.linalg.norm(closest_obj_pos - response_obj_pos)
        if dist < min_dist:
          min_dist = dist
          final_response = response
          final_name = cur_obj_name
    self.obj_name = final_name
    self.obj_state = final_response

  def place_object(self, obj_name):
    state = ModelState()
    state.model_name = obj_name
    state.reference_frame = "robot::base_link"

    pose = self.obj_state.pose

    pose.position.x = self.Goal_pose.position.x
    pose.position.y = self.Goal_pose.position.y
    pose.position.z += 0.01

    state.pose = pose
    self.set_model_state(state)

  def init_gripper(self):
    time.sleep(2)
    self.Grip.rACT = 0
    self.Grip.rPR = 0
    self.Grip.rGTO = 0
    self.Grip.rSP  = 255
    self.Grip.rFR = 0
    self.Grip.rATR = 0
    self.pub.publish(self.Grip)
    time.sleep(2)

    self.Grip.rACT = 1
    self.Grip.rPR = 0
    self.Grip.rGTO = 1
    self.Grip.rSP  = 255
    self.Grip.rFR = 150
    self.Grip.rATR = 0
    self.pub.publish(self.Grip)
    time.sleep(2)

  def init_home_pose(self):
    self.home_pose.position.x = -0.4
    self.home_pose.position.y = 0.1
    self.home_pose.position.z = 0.5
    self.home_pose.orientation.w = 0
    self.home_pose.orientation.x = -0.707106781
    self.home_pose.orientation.y = 0
    self.home_pose.orientation.z = 0.707106781

    self.Target_pose = copy.deepcopy(self.home_pose)
    self.Goal_pose   = copy.deepcopy(self.home_pose)

  def Gripper_close(self):
    self.Grip.rPR = 105
    self.pub.publish(self.Grip)
    time.sleep(2)

  def Gripper_open(self):
    self.Grip.rPR = 0
    self.pub.publish(self.Grip)
    time.sleep(2)
    
  def Cartesian_To_Pick(self):
    
    print("Pick")


    if not self.sim:
        self.Gripper_open()

    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose

    if not self.sim:
      wpose.position.x = self.Target_pose.position.x + 0.02
      waypoints.append(copy.deepcopy(wpose))
    else:  
      
      wpose.position.x = self.Target_pose.position.x
      waypoints.append(copy.deepcopy(wpose))

    wpose.position.y = self.Target_pose.position.y
    waypoints.append(copy.deepcopy(wpose))


    if not self.sim:
      wpose.position.z = gripperOffset
      waypoints.append(copy.deepcopy(wpose))
    else:
      wpose.position.z = gripperOffset_sim
      waypoints.append(copy.deepcopy(wpose))

    
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.05,        # eef_step
                                       0.0,         # jump_threshold
                                       True)

    success = False
    # Plan the trajectory
    if (fraction == 1):
      success = True
      group.execute(plan)
      time.sleep(1)

      if not self.sim:
          self.Gripper_close()
      else:
        #Vanish Object in Gazebo
        self.vanish_gazebo_object(self.obj_name)

    wpose.position.z = 0.4
    group.set_pose_target(wpose)

    plan = group.go(wait=True)

    print(group.get_current_pose().pose.position.x)
    print(group.get_current_pose().pose.position.y)
    print(group.get_current_pose().pose.position.z)

    return success

  def Cartesian_To_Place(self):
    print("Place")
    print(self.Goal_pose.position.x)
    print(self.Goal_pose.position.y)
    print(self.Goal_pose.position.z)

    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose

    #wpose.position.y = 0.1
    #waypoints.append(copy.deepcopy(wpose))

    wpose.position.y = self.Goal_pose.position.y
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = self.Goal_pose.position.x
    waypoints.append(copy.deepcopy(wpose))

    if not self.sim:
      wpose.position.z = gripperOffset
      waypoints.append(copy.deepcopy(wpose))
    else:
      wpose.position.z = gripperOffset_sim
      waypoints.append(copy.deepcopy(wpose))
    

    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.05,        # eef_step
                                       0.0,         # jump_threshold
                                       True)

    success = False
    # Plan the trajectory
    if (fraction == 1):
      success = True
      group.execute(plan)
      time.sleep(1)

      if not self.sim:
        self.Gripper_open()
      else:
        #Drop Object
        self.place_object(self.obj_name)



    wpose.position.z = 0.4
    group.set_pose_target(wpose)

    plan = group.go(wait=True)

    return success


  def go_home(self):
    print("Home")
    print("x", self.home_pose.position.x)
    print("y", self.home_pose.position.y)

    group = self.group
   
    waypoints = []
    wpose = group.get_current_pose().pose

    wpose.position.y = 0.1
    wpose.position.z = 0.4
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = self.home_pose.position.x
    waypoints.append(copy.deepcopy(wpose))

    wpose = self.home_pose
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.05,        # eef_step
                                       0.0,         # jump_threshold
                                       True)

    success = False
    print(fraction)

    # Plan the trajectory
    if (fraction == 1):
      success = True
      print("Home")

      group.execute(plan)
      time.sleep(1)

    return success

    
  def executeCB(self, goal):
    rospy.loginfo("executeCB: PickPlaceAction")

    ## Send Arm over the Object with some z offset
    self.Target_pose.position.x = goal.obj_centroid.point.x
    self.Target_pose.position.y = goal.obj_centroid.point.y

    self.Goal_pose.position.x = goal.placement.point.x
    self.Goal_pose.position.y = goal.placement.point.y
    self.Goal_pose.position.z = goal.placement.point.z

    if(self.sim):
      #Associate the centroid with one of the cylindrical objects in Gazebo
      self.get_closest_object(goal.obj_centroid.point)

    success = self.Cartesian_To_Pick()

    if(success):
      #Find placement position, move the arm there (random for now)

      success = self.Cartesian_To_Place()

      
      success = self.go_home()
    
    self.server.set_succeeded()
    
if __name__ == '__main__':
  rospy.init_node('pick_place_server')

  server = PickPlaceServer()
  rospy.spin()
