import rospy
from moveit_python import *

rospy.init_node("moveit_py")
g = MoveGroupInterface("planning_group_name", "base_link")
