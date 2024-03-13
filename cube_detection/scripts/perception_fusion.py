#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

import numpy as np

class Cube:
    def __init__(self, id, x, y, z, orientation):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.orientation = orientation


class CubeFusion:
    def __init__(self):
        self.cubes_pc = []
        self.cubes_ed = []

        self.matched_cubes = []

        for cube_num in range(6):
            self.cube_pc_subscriber = rospy.Subscriber("cube_{}_odom_pc".format(cube_num), Odometry, self.callbackPC)
            self.cube_ed_subscriber = rospy.Subscriber("cube_{}_odom_ed".format(cube_num), Odometry, self.callbackED)

    def callbackPC(self, data):
        id = data.child_frame_id[-1]
        rospy.loginfo("PC "+ id)

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z

        self.cubes_pc.append(Cube(id, x, y, z, None))

        self.run_matching()

    def callbackED(self, data):
        id = data.child_frame_id[-1]
        rospy.loginfo("ED "+ id)

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        orientation = data.pose.pose.orientation.z

        self.cubes_ed.append(Cube(id, x, y, z, orientation))

        self.run_matching()

    def run_matching(self):
        if len(self.cubes_pc) == 6 and len(self.cubes_ed) == 6:
            rospy.loginfo("Performing cube matching...")

            for cube_pc in self.cubes_pc:
                self.match_closest_cube(cube_pc)

            rospy.loginfo("Done with matching")
            self.cubes_pc = []
            self.cubes_ed = []

    def match_closest_cube(self, cube_pc):
        distances = []
        for cube_ed in self.cubes_ed:
            distances.append((cube_ed.id, self.calculate_distance(cube_pc, cube_ed)))

        print(distances)

        #coninue here
        #closest = np.argmin(distances[])

    def calculate_distance(self, cube1, cube2):
        # Euclidean distance between two cubes
        return ((cube1.x - cube2.x) ** 2 + (cube1.y - cube2.y) ** 2)


    def publisher(self):
        try:
            # Publish cubes with different topic names
            for i in range(6):  # Assuming you want to publish 28 cubes                
                cube_odom = Odometry()
                # Populate cube_odom message with necessary data, assuming you have them stored in parameters
                cube_odom.header.frame_id = "world"
                cube_odom.child_frame_id = "cube_{}".format(i)
                cube_odom.pose.pose.position.x = rospy.get_param("cube_"+str(i)+"_x", default=0)
                cube_odom.pose.pose.position.y = rospy.get_param("cube_"+str(i)+"_y", default=0)
                cube_odom.pose.pose.position.z = rospy.get_param("cube_"+str(i)+"_z", default=0)
                cube_odom.pose.pose.orientation.x = rospy.get_param("cube_"+str(i)+"_orient_x", default=0)
                cube_odom.pose.pose.orientation.y = rospy.get_param("cube_"+str(i)+"_orient_y", default=0)
                cube_odom.pose.pose.orientation.z = rospy.get_param("cube_"+str(i)+"_orient_z", default=0)
                cube_odom.pose.pose.orientation.w = rospy.get_param("cube_"+str(i)+"_orient_w", default=0)
                # Publish cube_odom message
                topic_name = "cube_" + str(i) + "_odom"
                rospy.loginfo("Publishing " + topic_name)
                pub = rospy.Publisher(topic_name, Odometry, queue_size=10)
                pub.publish(cube_odom)
        except Exception as e:
            print(e)
        


if __name__ == '__main__':
    rospy.init_node('perception_fusion', anonymous=True)
    cube_fusion = CubeFusion()
    rospy.spin()
