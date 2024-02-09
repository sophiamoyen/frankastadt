#!/usr/bin/env python3

import rospy
from pick_and_place_module.pick_and_place import PickAndPlace
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import *

def task():
    pick_and_place = PickAndPlace(0.05, 0.5)

    """ 
    --- Picking ---
    Assumes that z axis is always turned upwards! 
    """
    """
    pick_position=[rospy.get_param("cube_0_x"),rospy.get_param("cube_0_y"),rospy.get_param("cube_0_z")+0.05]
    """
    """
    cube_orientation = list(euler_from_quaternion([rospy.get_param("cube_0_orient_x"),
                                                    rospy.get_param("cube_0_orient_y"),
                                                    rospy.get_param("cube_0_orient_z"),
                                                    rospy.get_param("cube_0_orient_w")]))
    orient_x = cube_orientation[0]%pi
    orient_y = cube_orientation[1]%pi
    orient_z = cube_orientation[2]%pi

    if orient_z>(pi/2) and orient_z<(3*pi/2):
        orient_x = orient_x - pi
        orient_y = orient_y - pi
        orient_z = orient_z - pi
    """
    pick_position = [0.4,0.2,0.2]
    pick_orientation=[1.57,3.14,0]

    """ 
    --- Placing ---
    """
    #place_position=[rospy.get_param("cube_0_x")-0.05,rospy.get_param("cube_0_y")-0.05,rospy.get_param("cube_0_z")+0.1]
    place_position = [0.3,0.15,0.2]
    place_orientation=[1.57,3.14,0]
    """
    --- Execution ---
    """
    pick_and_place.setPickPose(*pick_position,*pick_orientation)
    pick_and_place.setDropPose(*place_position,*place_orientation)
    pick_and_place.setGripperPose(0.01, 0.01)

    pick_and_place.execute_pick_and_place()

if __name__ == "__main__":
    task()