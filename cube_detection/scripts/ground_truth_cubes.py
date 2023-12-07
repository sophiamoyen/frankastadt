#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import time 

def cube_ground_truth():
    rospy.init_node('cube_ground_truth', anonymous=True)
    cube_0 = rospy.Subscriber("/cube_0_odom", Odometry, cube_callback)
    cube_1 = rospy.Subscriber("/cube_1_odom", Odometry, cube_callback)
    rospy.sleep(10000000000)

def cube_callback(odometry):
    print(odometry)
    time.sleep(1000000)

if __name__ == '__main__':
    cube_ground_truth()