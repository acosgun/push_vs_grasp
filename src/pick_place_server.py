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
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import *
import time
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from push_vs_grasp.msg import PickPlaceAction, PickPlaceResult
from control_msgs.msg import *
from trajectory_msgs.msg import *

#GLOBAL PARAMS
gripperOffset = 0.3
gripperOffset_sim = 0.32
jump_threshold = 5 #5
max_num_objs = 10
eef_step = 0.01
model_name = "unit_cylinder"

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
#Q1 = [2.2,0,-1.57,0,0,0]
#Q2 = [1.5,0,-1.57,0,0,0]
#Q3 = [1.5,-0.2,-1.57,0,0,0]
 
q_home_push_sim = [0.022094493160206063, -1.693481450827033, -1.7637408523401323, 3.8549434891245378, 1.5700411220305215, 0.0007686897581802299]
q_home_push_real = [0.022597806250636232, -1.5856213618880979, -1.5743019037965347, 3.1577767221509427, 1.5703875993524257, 0.0008091528403513237]

q_home_pick_real = [0.023, -1.38, -1.57, 4.51, 1.57, 0.0]
q_home_pick_sim = [0.02236895846625675, -1.2769328818447914, -2.139559890001145, 4.9834799025128875, 1.5699749396844025, -0.0004364574301582991]

class PickPlaceServer:
  def __init__(self):
    #Gazebo Simluation

    self.sim = rospy.myargv(argv=sys.argv)[1]
    rospy.loginfo(self.sim)

    self.init_moveit()
    rospy.loginfo("init_moveit ")

    self.init_gazebo()
    rospy.loginfo("init_gazebo ")

    self.init_real()
    rospy.loginfo("init_real ")

    self.obj_name ="n/a"
    self.obj_state = 'n/a'

    self.numPushes = 0
    self.numPickPlaces = 0

    self.home_pose = geometry_msgs.msg.Pose()
    self.Goal_pose = geometry_msgs.msg.Pose()
    self.Target_pose = geometry_msgs.msg.Pose()
    rospy.loginfo("Target_pose ")

    self.server = actionlib.SimpleActionServer('pick_place', PickPlaceAction, self.executeCB, False)        
    rospy.loginfo("server ")

    self.follow_joint_trajectory_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    print "Waiting for follow_joint_trajectory server..."
    self.follow_joint_trajectory_client.wait_for_server()
    print "Connected to follow_joint_trajectory server"
        
    self.init_home_pose()
    rospy.loginfo("init_home_pose ")

    self.go_home(for_pushing = True)
    rospy.loginfo("go_home ")

    self.server.start()
    rospy.loginfo("Pick Place Server ON")  

  def init_moveit(self):

    robot = moveit_commander.RobotCommander()

    print "self.sim: "  + str(self.sim)
    
    if self.sim == "true":
      self.sim = True
      self.gripper_offset = gripperOffset_sim
    else:
      self.gripper_offset = gripperOffset
      self.sim = False

    scene = moveit_commander.PlanningSceneInterface("base_link")

    #time.sleep(1)

    # Create table obstacle
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    #print p.header.frame_id
    p.pose.position.x = -0.5
    p.pose.position.y = 0
    p.pose.position.z = -0.03
    scene.add_box("table", p, (2, 2, 0.06))
    #time.sleep(1)

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
      else:
        break

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
    time.sleep(1)
    self.Grip.rACT = 0
    self.Grip.rPR = 0
    self.Grip.rGTO = 0
    self.Grip.rSP  = 255
    self.Grip.rFR = 0
    self.Grip.rATR = 0
    self.pub.publish(self.Grip)
    time.sleep(1)

    self.Grip.rACT = 1
    self.Grip.rPR = 0
    self.Grip.rGTO = 1
    self.Grip.rSP  = 255
    self.Grip.rFR = 150
    self.Grip.rATR = 0
    self.pub.publish(self.Grip)
    time.sleep(1)

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
    self.Grip.rPR = 100
    self.pub.publish(self.Grip)
    time.sleep(1)

  def Gripper_open(self):
    self.Grip.rPR = 50
    self.pub.publish(self.Grip)
    time.sleep(1)

  def Cartesian_To_Pick(self):

    print("Pick")
    if not self.sim:
        self.Gripper_open()

    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x = self.Target_pose.position.x
    wpose.position.y = self.Target_pose.position.y
    wpose.position.z = self.gripper_offset + 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = self.gripper_offset
    waypoints.append(copy.deepcopy(wpose))


    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       eef_step,        # eef_step
                                       jump_threshold,         # jump_threshold
                                       True)
    success = False
    # Plan the trajectory
    if (fraction == 1):
      success = True
      group.execute(plan)

      if not self.sim:
          self.Gripper_close()
      else:
        #Vanish Object in Gazebo
        self.vanish_gazebo_object(self.obj_name)
        time.sleep(1)
        

    wpose.position.z = 0.5
    group.set_pose_target(wpose)

    plan = group.go(wait=True)

    #print(group.get_current_pose().pose.position.x)
    #print(group.get_current_pose().pose.position.y)
    #print(group.get_current_pose().pose.position.z)

    return success

  def Cartesian_To_Place(self):
    print("Place")
    self.numPickPlaces = self.numPickPlaces + 1
    #print(self.Goal_pose.position.x)
    #print(self.Goal_pose.position.y)
    #print(self.Goal_pose.position.z)

    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose

    #wpose.position.y = 0.1
    #waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = self.Goal_pose.position.x
    wpose.position.y = self.Goal_pose.position.y
    wpose.position.z = self.gripper_offset + 0.2
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = self.gripper_offset + 0.02
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       eef_step,        # eef_step
                                       jump_threshold,         # jump_threshold
                                       True)

    success = False
    # Plan the trajectory
    if (fraction == 1):
      success = True
      group.execute(plan)

      if not self.sim:
        self.Gripper_open()
      else:
        #Drop Object
        self.place_object(self.obj_name)
        time.sleep(1)



    wpose.position.z = 0.4
    group.set_pose_target(wpose)

    plan = group.go(wait=True)

    return success

  def go_home(self, for_pushing):
    print("go_home")

    if self.sim:
      if for_pushing:
        self.send_robot_to_joint_state(q_home_push_sim)
      else:
        self.send_robot_to_joint_state(q_home_pick_sim)
    else:
      if for_pushing:
        self.send_robot_to_joint_state(q_home_push_real)
      else:
        self.send_robot_to_joint_state(q_home_pick_real)

    return True
  
    #group = self.group

    #waypoints = []
    #wpose = self.home_pose
    #waypoints.append(copy.deepcopy(wpose))

    #(plan, fraction) = group.compute_cartesian_path(
    #                                   waypoints,   # waypoints to follow
    #                                   eef_step,        # eef_step
    #                                   jump_threshold,         # jump_threshold
    #                                   True)

    #success = False
    #if (fraction > 0):
    #  success = True
    #  group.execute(plan)
    #return success

  def lift_ee_up(self, delta):
    print('lift_ee_up')
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.z = wpose.position.z + delta
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       eef_step,        # eef_step
                                       jump_threshold,         # jump_threshold
                                       True)
    success = False    
    if fraction > 0:
      success = True
      group.execute(plan)
    return success    

    
  def execute_push(self, pos):
    print('execute_push')
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x = pos.x
    wpose.position.y = pos.y
    waypoints.append(copy.deepcopy(wpose))
    
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       eef_step,        # eef_step
                                       jump_threshold,         # jump_threshold
                                       True)    

    success = False    
    if fraction > 0:
      success = True
      group.execute(plan)
    return fraction


  def get_cartesian_plan(self, pos, quat_1):
    group = self.group
    z_high = 0.15
    z_low = 0.07

    waypoints = []

    # WP 1
    wpose = group.get_current_pose().pose
    wpose.orientation.x = quat_1[0]
    wpose.orientation.y = quat_1[1]
    wpose.orientation.z = quat_1[2]
    wpose.orientation.w = quat_1[3]
    #waypoints.append(copy.deepcopy(wpose))
    
    # WP 2
    wpose.position.x = pos.x
    wpose.position.y = pos.y
    wpose.position.z = z_high
    waypoints.append(copy.deepcopy(wpose))

    # WP 3
    wpose.position.z = z_low
    waypoints.append(copy.deepcopy(wpose))

    (plan_1, fraction_1) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       eef_step,        # eef_step
                                       jump_threshold,         # jump_threshold
                                       True)    

    return plan_1, fraction_1


  def send_robot_to_joint_state(self, q):
    print('send_robot_to_joint_state')
    current_joint_state = self.group.get_current_joint_values()    
    g = FollowJointTrajectoryGoal()    
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
    self.follow_joint_trajectory_client.send_goal(g)
    self.follow_joint_trajectory_client.wait_for_result()    
    return
  
  def go_to_pre_push_pose(self, pos, quat_1, quat_2):
    print('go_to_pre_push_pose')
    
    [plan_1, fraction_1] = self.get_cartesian_plan(pos, quat_1)
    [plan_2, fraction_2] = self.get_cartesian_plan(pos, quat_2)

    #TODO: get closest to the end effector orientation
    
    success = False  
    if (fraction_1 == 1):
      success = True
      self.group.execute(plan_1)
    elif (fraction_2 == 1):
      success = True
      self.group.execute(plan_2)

    return success    

  def limit_angle_pi_minus_pi(self, angle):
    if angle > pi:
      angle = angle - pi
    elif angle < pi:
      angle = angle + pi
    return angle
  
  def compute_push_orientation(self, p1, p2):    
    x_diff = p2.x - p1.x
    y_diff = p2.y - p1.y
    from math import atan2
    angle = atan2(y_diff,x_diff)
    #print "angle: " + str(angle)
    #GradM = Ydiff/Xdiff
    #Yoffset = p2.y - GradM * p2.x;
    #import numpy as np 
    #Zangle = - np.tanh(1/GradM);
    
    quat_1 = quaternion_from_euler(pi, 0, self.limit_angle_pi_minus_pi(angle-pi/2))
    quat_2 = quaternion_from_euler(pi, 0, self.limit_angle_pi_minus_pi(angle+pi/2))
    return quat_1, quat_2
    
  def executeCB(self, goal):
    rospy.loginfo("executeCB: PickPlaceAction")
      
    self.Target_pose.position.x = goal.obj_centroid.point.x
    self.Target_pose.position.y = goal.obj_centroid.point.y
    self.Target_pose.position.z = goal.obj_centroid.point.z

    self.Goal_pose.position.x = goal.placement.point.x
    self.Goal_pose.position.y = goal.placement.point.y
    self.Goal_pose.position.z = goal.placement.point.z          

    result = PickPlaceResult()
    result.fraction = 0.0
    
    if goal.action_type == 0: #Push Action
      self.go_home(for_pushing=True)
      [quat_1, quat_2] = self.compute_push_orientation(goal.obj_centroid.point, goal.placement.point)      
      success = self.go_to_pre_push_pose(goal.obj_centroid.point, quat_1, quat_2)
      if success:
        fraction = self.execute_push(goal.placement.point)
        if fraction > 0:
          result.fraction = fraction
          success = self.lift_ee_up(0.1)
          success = self.go_home(for_pushing=True)
          self.server.set_succeeded(result)
        else:
          print "execute_push FAILED"
          result.fraction = 0.0
          self.server.set_aborted(result)
      else:
        print "go_to_pre_push_pose FAILED"
        result.fraction = 0.0
        self.server.set_aborted(result)
                
    elif goal.action_type == 1: # Pick Action
      self.go_home(for_pushing=False)
      if(self.sim):
        #Associate the centroid with one of the cylindrical objects in Gazebo
        self.get_closest_object(goal.obj_centroid.point)

        success = self.Cartesian_To_Pick()
        if(success):
          success1 = self.Cartesian_To_Place()
          success2 = self.go_home(for_pushing=False)

        result.fraction = 1.0
        self.server.set_succeeded(result)


if __name__ == '__main__':
  rospy.init_node('pick_place_server')

  server = PickPlaceServer()
  rospy.spin()
