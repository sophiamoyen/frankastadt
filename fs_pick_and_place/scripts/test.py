#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("test", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# Planning Scene
"""
scene.remove_world_object("left_bar")
scene.remove_world_object("right_bar")
scene.remove_world_object("front_bar")
scene.remove_world_object("table")
"""

table_pose = geometry_msgs.msg.PoseStamped()
table_pose.header.frame_id = "world"
table_pose.pose.orientation.w = 1.0
table_pose.pose.position.x = 0.495
table_pose.pose.position.y = 0
table_pose.pose.position.z = -0.3935
scene.add_box("table", table_pose, size=(0.81, 1.49, 0.787))

left_bar_pose = geometry_msgs.msg.PoseStamped()
left_bar_pose.header.frame_id = "world"
left_bar_pose.pose.orientation.w = 1.0
left_bar_pose.pose.position.x = 0.495
left_bar_pose.pose.position.y = 0.795
left_bar_pose.pose.position.z = 0.4
scene.add_box("left_bar", left_bar_pose, size=(0.81, 0.1, 1.6))


right_bar_pose = geometry_msgs.msg.PoseStamped()
right_bar_pose.header.frame_id = "world"
right_bar_pose.pose.orientation.w = 1.0
right_bar_pose.pose.position.x = 0.495
right_bar_pose.pose.position.y = -0.795
right_bar_pose.pose.position.z = 0.4
scene.add_box("right_bar", right_bar_pose, size=(0.81, 0.1, 1.6))

front_bar_pose = geometry_msgs.msg.PoseStamped()
front_bar_pose.header.frame_id = "world"
front_bar_pose.pose.orientation.w = 1.0
front_bar_pose.pose.position.x = 0.86
front_bar_pose.pose.position.y = 0
front_bar_pose.pose.position.z = 0.4
scene.add_box("front_bar", front_bar_pose, size=(0.1, 1.49, 1.6))


group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
                                "/move_group/display_planned_path",
                                moveit_msgs.msg.DisplayTrajectory,
                                queue_size=20,
                            )

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing move group pose")
print(move_group.get_current_pose())
print("")

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing move group joint values")
print(move_group.get_current_joint_values())
print("")

"""
# We get the joint values from the group and change some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -tau / 8
joint_goal[2] = 0
joint_goal[3] = -tau / 4
joint_goal[4] = 0
joint_goal[5] = tau / 6  # 1/6 of a turn
joint_goal[6] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()
"""