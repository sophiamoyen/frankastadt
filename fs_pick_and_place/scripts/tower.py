#!/usr/bin/env python3

import rospy
from pick_and_place_module.pick_and_place import PickAndPlace
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import *
import geometry_msgs.msg

class Tower():
    def __init__(self):
        self.pick_and_place = PickAndPlace(0.05, 0.5)

    def collision_scene(self):
        """
        self.pick_and_place.moveit_control.scene.remove_world_object("left_bar")


        # Gazebo Collision
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "world"
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.x = 0.495
        table_pose.pose.position.y = 0
        table_pose.pose.position.z = 0.3935
        self.pick_and_place.moveit_control.scene.add_box("table", table_pose, size=(0.81, 1.49, 0.787))

        left_bar_pose = geometry_msgs.msg.PoseStamped()
        left_bar_pose.header.frame_id = "world"
        left_bar_pose.pose.orientation.w = 1.0
        left_bar_pose.pose.position.x = 0.495
        left_bar_pose.pose.position.y = 0.795
        left_bar_pose.pose.position.z = 0.81
        self.pick_and_place.moveit_control.scene.add_box("left_bar", left_bar_pose, size=(0.81, 0.1, 1.6))


        right_bar_pose = geometry_msgs.msg.PoseStamped()
        right_bar_pose.header.frame_id = "world"
        right_bar_pose.pose.orientation.w = 1.0
        right_bar_pose.pose.position.x = 0.495
        right_bar_pose.pose.position.y = -0.795
        right_bar_pose.pose.position.z = 0.81
        self.pick_and_place.moveit_control.scene.add_box("right_bar", right_bar_pose, size=(0.81, 0.1, 1.6))

        front_bar_pose = geometry_msgs.msg.PoseStamped()
        front_bar_pose.header.frame_id = "world"
        front_bar_pose.pose.orientation.w = 1.0
        front_bar_pose.pose.position.x = 0.86
        front_bar_pose.pose.position.y = 0
        front_bar_pose.pose.position.z = 0.81
        self.pick_and_place.moveit_control.scene.add_box("front_bar", front_bar_pose, size=(0.1, 1.49, 1.6))
        """

        # Real Robot Collision
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "world"
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.x = 0.495
        table_pose.pose.position.y = 0
        table_pose.pose.position.z = -0.3935
        self.pick_and_place.moveit_control.scene.add_box("table", table_pose, size=(0.81, 1.49, 0.787))

        left_bar_pose = geometry_msgs.msg.PoseStamped()
        left_bar_pose.header.frame_id = "world"
        left_bar_pose.pose.orientation.w = 1.0
        left_bar_pose.pose.position.x = 0.495
        left_bar_pose.pose.position.y = 0.795
        left_bar_pose.pose.position.z = 0.4
        self.pick_and_place.moveit_control.scene.add_box("left_bar", left_bar_pose, size=(0.81, 0.1, 1.6))


        right_bar_pose = geometry_msgs.msg.PoseStamped()
        right_bar_pose.header.frame_id = "world"
        right_bar_pose.pose.orientation.w = 1.0
        right_bar_pose.pose.position.x = 0.495
        right_bar_pose.pose.position.y = -0.795
        right_bar_pose.pose.position.z = 0.4
        self.pick_and_place.moveit_control.scene.add_box("right_bar", right_bar_pose, size=(0.81, 0.1, 1.6))

        front_bar_pose = geometry_msgs.msg.PoseStamped()
        front_bar_pose.header.frame_id = "world"
        front_bar_pose.pose.orientation.w = 1.0
        front_bar_pose.pose.position.x = 0.86
        front_bar_pose.pose.position.y = 0
        front_bar_pose.pose.position.z = 0.4
        self.pick_and_place.moveit_control.scene.add_box("front_bar", front_bar_pose, size=(0.1, 1.49, 1.6))

    def stack(self,n_cube, place_position):
        """ 
        --- Picking ---
        Assumes that z axis is always turned upwards! 
        """
        pick_position = [rospy.get_param("cube_{}_x".format(n_cube)),
                    rospy.get_param("cube_{}_y".format(n_cube)),
                    rospy.get_param("cube_{}_z".format(n_cube))+0.05]
        pick_orientation = [1.57,3.14,0]

        """ 
        --- Placing ---
        """
        place_position = place_position
        place_orientation = [1.57,3.14,0]
        
        """
        --- Execution ---
        """
        self.pick_and_place.setPickPose(*pick_position,*pick_orientation)
        self.pick_and_place.setDropPose(*place_position,*place_orientation)
        self.pick_and_place.setGripperPose(0.01, 0.01)

        self.pick_and_place.execute_pick_and_place()

    def task(self):
        cube0_position = [0.5,0,rospy.get_param("cube_0_z")+0.1]
        cube1_position = [0.5,0.045,rospy.get_param("cube_0_z")+0.1]
        cube2_position = [0.5,-0.045,rospy.get_param("cube_0_z")+0.1]
        cube3_position = [0.5,-0.0225,rospy.get_param("cube_0_z")+0.15]
        cube4_position = [0.5,0.0225,rospy.get_param("cube_0_z")+0.15]
        cube5_position = [0.5,0,rospy.get_param("cube_0_z")+0.2]

        list_cubes = [cube0_position,
                    cube1_position,
                    cube2_position,
                    cube3_position,
                    cube4_position,
                    cube5_position]

        i = 0
        for cube_pos in list_cubes:
            self.stack(i,cube_pos)
            i += 1

    def test(self):
        pick_position = [0.4,0.2,0.2]
        pick_orientation = [1.57,3.14,0]
        place_position = [0.5,0.15,0.2]
        place_orientation = [1.57,3.14,0]

        self.pick_and_place.setPickPose(*pick_position,*pick_orientation)
        self.pick_and_place.setDropPose(*place_position,*place_orientation)
        self.pick_and_place.setGripperPose(0.01, 0.01)

        self.pick_and_place.execute_pick_and_place()

if __name__ == "__main__":
    cla = Tower()
    cla.collision_scene()
    cla.test()