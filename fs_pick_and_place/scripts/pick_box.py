#!/usr/bin/env python3

import rospy
from pick_and_place_module.pick_and_place import PickAndPlace

def task():
    pick_and_place = PickAndPlace(0.05, 0.5)
    pick_position=[rospy.get_param("cube_0_x"),rospy.get_param("cube_0_y"),rospy.get_param("cube_0_z")+0.05]
    pick_orientation=[1.57,3.14,0]

    place_position=[rospy.get_param("cube_0_x")+0.05,rospy.get_param("cube_0_y")+0.2,rospy.get_param("cube_0_z")+0.1]
    place_orientation=[1.57,3.14,0]
    
    pick_and_place.setPickPose(*pick_position,*pick_orientation)
    pick_and_place.setDropPose(*place_position,*place_orientation)
    pick_and_place.setGripperPose(0.01, 0.01)

    pick_and_place.execute_pick_and_place()

if __name__ == "__main__":
    task()
