#!/usr/bin/env python3

from tower import Tower
import rospy

if __name__ == "__main__":
    cla = Tower()

    # Open gripper
    cla.plan_and_move.gripper.move(0.04, 0.04)
    rospy.sleep(2)
    cla.plan_and_move.move_standard_pose()