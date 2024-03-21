#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

def callback(data):

    index=data.child_frame_id[-1]
    rospy.set_param("cube_"+str(index)+"_x",data.pose.pose.position.x)
    rospy.set_param("cube_"+str(index)+"_y",data.pose.pose.position.y)
    rospy.set_param("cube_"+str(index)+"_z",data.pose.pose.position.z)
    rospy.set_param("cube_"+str(index)+"_orient_x",data.pose.pose.orientation.x)
    rospy.set_param("cube_"+str(index)+"_orient_y",data.pose.pose.orientation.y)
    rospy.set_param("cube_"+str(index)+"_orient_z",data.pose.pose.orientation.z)
    rospy.set_param("cube_"+str(index)+"_orient_w",data.pose.pose.orientation.w)
    
def callback_cube_num(data):
    rospy.set_param("num_cubes",data.data)

def callback_tower_state(data):
    rospy.set_param("pyramid_state",data.data)

def callback_tower_odom(data):
    rospy.set_param("pyramid_x",data.pose.pose.position.x)
    rospy.set_param("pyramid_y",data.pose.pose.position.y)

    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("num_cubes",String , callback_cube_num)
    rospy.Subscriber("pyramid_state",String , callback_tower_state)
    rospy.Subscriber("pyramid_odom", Odometry, callback_tower_odom)
    rospy.Subscriber("cube_0_odom", Odometry, callback)
    rospy.Subscriber("cube_1_odom", Odometry, callback)
    rospy.Subscriber("cube_2_odom", Odometry, callback)
    rospy.Subscriber("cube_3_odom", Odometry, callback)
    rospy.Subscriber("cube_4_odom", Odometry, callback)
    rospy.Subscriber("cube_5_odom", Odometry, callback)
    rospy.Subscriber("cube_6_odom", Odometry, callback)
    rospy.Subscriber("cube_7_odom", Odometry, callback)
    rospy.Subscriber("cube_8_odom", Odometry, callback)
    rospy.Subscriber("cube_9_odom", Odometry, callback)
    """
    rospy.Subscriber("cube_10_odom", Odometry, callback)
    rospy.Subscriber("cube_11_odom", Odometry, callback)
    rospy.Subscriber("cube_12_odom", Odometry, callback)
    rospy.Subscriber("cube_13_odom", Odometry, callback)
    rospy.Subscriber("cube_14_odom", Odometry, callback)
    rospy.Subscriber("cube_15_odom", Odometry, callback)
    rospy.Subscriber("cube_16_odom", Odometry, callback)
    rospy.Subscriber("cube_17_odom", Odometry, callback)
    rospy.Subscriber("cube_18_odom", Odometry, callback)
    rospy.Subscriber("cube_19_odom", Odometry, callback)
    rospy.Subscriber("cube_20_odom", Odometry, callback)
    rospy.Subscriber("cube_21_odom", Odometry, callback)
    rospy.Subscriber("cube_22_odom", Odometry, callback)
    rospy.Subscriber("cube_23_odom", Odometry, callback)
    rospy.Subscriber("cube_24_odom", Odometry, callback)
    rospy.Subscriber("cube_25_odom", Odometry, callback)
    rospy.Subscriber("cube_26_odom", Odometry, callback)
    rospy.Subscriber("cube_27_odom", Odometry, callback)
    """

    rospy.spin()

if __name__ == '__main__':
    listener()
