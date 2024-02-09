#!/usr/bin/env python3

import rospy
from pick_and_place_module.pick_and_place import PickAndPlace
from pick_and_place_module.move_functions import PlanAndMove
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import *
import geometry_msgs.msg

class Tower():
    def __init__(self):
        self.pick_and_place = PickAndPlace(0.05, 0.5)
        self.plan_and_move = PlanAndMove()

    def pickplace(self):
        self.plan_and_move.move_standard_pose()
        
        
        pick_position = [rospy.get_param("cube_0_x")-0.01,
                        rospy.get_param("cube_0_y")-0.05,
                        0.05]
        """
        pick_position = [0.48, -0.32, 0.04]
        """
        print("pick_position",pick_position)
        

        pick_orientation = [0.9239002820650952,  
                            -0.3826324133679813, 
                            -0.000784053224384248,  
                            0.00030050087016984296]

        place_position = [rospy.get_param("cube_1_x")-0.01,
                          rospy.get_param("cube_1_y")-0.05,
                          0.12]

        self.plan_and_move.setPickPose(*pick_position,*pick_orientation)
        self.plan_and_move.setPlacePose(*place_position,*pick_orientation)
        self.plan_and_move.execute_pick()
        self.plan_and_move.execute_place()
        self.plan_and_move.move_standard_pose()

    def collision_scene(self):
        # Creates collision scene for table and around the robot
        
        """
        self.plan_and_move.moveit_control.scene.remove_world_object("left_bar")
        self.plan_and_move.moveit_control.scene.remove_world_object("right_bar")
        self.plan_and_move.moveit_control.scene.remove_world_object("front_bar")
        self.plan_and_move.moveit_control.scene.remove_world_object("table")
        """

        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "panda_link0"
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.x = 0.495
        table_pose.pose.position.y = 0
        table_pose.pose.position.z = -0.3935
        self.plan_and_move.moveit_control.scene.add_box("table", table_pose, size=(0.81, 1.49, 0.787))

        left_bar_pose = geometry_msgs.msg.PoseStamped()
        left_bar_pose.header.frame_id = "panda_link0"
        left_bar_pose.pose.orientation.w = 1.0
        left_bar_pose.pose.position.x = 0.495
        left_bar_pose.pose.position.y = 0.795
        left_bar_pose.pose.position.z = 0.4
        self.plan_and_move.moveit_control.scene.add_box("left_bar", left_bar_pose, size=(0.81, 0.1, 1.6))

        right_bar_pose = geometry_msgs.msg.PoseStamped()
        right_bar_pose.header.frame_id = "panda_link0"
        right_bar_pose.pose.orientation.w = 1.0
        right_bar_pose.pose.position.x = 0.495
        right_bar_pose.pose.position.y = -0.795
        right_bar_pose.pose.position.z = 0.4
        self.plan_and_move.moveit_control.scene.add_box("right_bar", right_bar_pose, size=(0.81, 0.1, 1.6))

        front_bar_pose = geometry_msgs.msg.PoseStamped()
        front_bar_pose.header.frame_id = "panda_link0"
        front_bar_pose.pose.orientation.w = 1.0
        front_bar_pose.pose.position.x = 0.86
        front_bar_pose.pose.position.y = 0
        front_bar_pose.pose.position.z = 0.4
        self.plan_and_move.moveit_control.scene.add_box("front_bar", front_bar_pose, size=(0.1, 1.49, 1.6))
        
    def stack(self, n_cube, place_position,pick_position):
        self.plan_and_move.move_standard_pose()
        """ 
        --- Picking ---
        Assumes that z axis is always turned upwards! 
        """
        pick_position = pick_position

        pick_orientation = [0.9239002820650952,  
                            -0.3826324133679813, 
                            -0.000784053224384248,  
                            0.00030050087016984296]

        """ 
        --- Placing ---
        """
        place_position = place_position
        
        """
        --- Execution ---
        """
        self.plan_and_move.setPickPose(*pick_position,*pick_orientation)
        self.plan_and_move.setPlacePose(*place_position,*pick_orientation)
        self.plan_and_move.execute_pick()
        self.plan_and_move.execute_place()

    def task_3_cubes(self):
        cube0_position = [0.5-0.01,-0.05,0.05]
        cube1_position = [0.55-0.01,-0.05,0.05]
        cube2_position = [0.53-0.01,-0.05,0.11]

        list_cubes_pos = []
        
        for i in range(3):
            pick_position = [rospy.get_param("cube_{}_x".format(i))-0.01,
                            rospy.get_param("cube_{}_y".format(i))-0.05,
                            0.04]
            list_cubes_pos.append(pick_position)

        list_cubes = [cube0_position,
                      cube1_position,
                      cube2_position]

        self.plan_and_move.move_standard_pose()

        i = 0
        for cube_pos in list_cubes:
            self.stack(i,cube_pos,list_cubes_pos[i])
            i += 1

        self.plan_and_move.move_standard_pose()

    def task(self):
        cube0_position = [0.5,0,rospy.get_param("cube_0_z")]
        cube1_position = [0.545,0,rospy.get_param("cube_0_z")]
        cube2_position = [0.455,0,rospy.get_param("cube_0_z")]
        cube3_position = [0.4775,0,rospy.get_param("cube_0_z")+0.05]
        cube4_position = [0.5225,0,rospy.get_param("cube_0_z")+0.05]
        cube5_position = [0.5,0,rospy.get_param("cube_0_z")+0.1]

        list_cubes = [cube0_position,
                    cube1_position,
                    cube2_position,
                    cube3_position,
                    cube4_position,
                    cube5_position]

        self.plan_and_move.move_standard_pose()

        i = 0
        for cube_pos in list_cubes:
            self.stack(i,cube_pos)
            i += 1

        self.plan_and_move.move_standard_pose()


if __name__ == "__main__":
    cla = Tower()
    cla.collision_scene()
    cla.task_3_cubes()
    #cla.task()


