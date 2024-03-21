#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

import numpy as np
import math

DISTANCE_THRESHOLD = 0.05
NUM_OF_CUBES = int(rospy.get_param("num_cubes"))
PREV_CUBE_MATCH_DISTANCE = 0.05

class Cube:
    def __init__(self, id, x, y, z, orientation, confidence=None):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.orientation = orientation
        self.confidence = confidence


class CubeFusion:
    def __init__(self):
        self.cubes_pc = []
        self.cubes_ed = []

        self.matched_cubes = []
        self.prev_cubes = []

        for cube_num in range(NUM_OF_CUBES):
            self.cube_pc_subscriber = rospy.Subscriber("cube_{}_odom_pc".format(cube_num), Odometry, self.callbackPC)
            self.cube_ed_subscriber = rospy.Subscriber("cube_{}_odom_ed".format(cube_num), Odometry, self.callbackED)

    def callbackPC(self, data):
        id = data.child_frame_id[-1]
        #rospy.loginfo("PC "+ id)

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z

        self.cubes_pc.append(Cube(id, x, y, z, None))

        self.run_matching()

        if (len(self.cubes_pc) > NUM_OF_CUBES):
            self.cubes_pc.clear()

    def callbackED(self, data):
        id = data.child_frame_id[-1]
        #rospy.loginfo("ED "+ id)

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        orientation = data.pose.pose.orientation.z

        self.cubes_ed.append(Cube(id, x, y, z, orientation))

        self.run_matching()
        
        if (len(self.cubes_ed) > NUM_OF_CUBES):
            self.cubes_ed.clear()
    

    def run_matching(self):
        if len(self.cubes_pc) == NUM_OF_CUBES and len(self.cubes_ed) == NUM_OF_CUBES:
            rospy.loginfo("Performing cube matching...")

            for cube_pc in self.cubes_pc:
                min_distance, closest_cube_ed = self.match_closest_cube(cube_pc)
                if (min_distance < DISTANCE_THRESHOLD):
                    self.matched_cubes.append((Cube(cube_pc.id, cube_pc.x, cube_pc.y, cube_pc.z, closest_cube_ed.orientation, 1)))
                else:
                    # find logic for cubes that the rgbd edge detection didnt find
                    self.matched_cubes.append((Cube(cube_pc.id, cube_pc.x, cube_pc.y, cube_pc.z, closest_cube_ed.orientation, 0)))

            #if (self.prev_cubes):
                #self.match_with_prev()
                # copy to prev
                # delete matched
            #else:
            self.publish_cubes()

            rospy.loginfo("Done with matching")
            self.cubes_pc = []
            self.cubes_ed = []

    def convert_rotation(self, angle):
        """
        Converts a cube's rotation angle in 2D to a value between 0 and 45 degrees.

        Args:
            angle: The rotation angle in degrees (or radians).

        Returns:
            The converted angle between 0 and 45 degrees (or pi/4 radians).
        """
        # Normalize the angle to be between 0 and 360 degrees (or 2*pi radians)
        angle = angle % 90

        #print("converted_angle",angle)
        return angle * math.pi / 180  # Convert to radians if needed

    def match_closest_cube(self, cube_pc):
        min_distance = float('inf')
        closest_cube_ed = None

        for cube_ed in self.cubes_ed:
            distance = self.calculate_distance(cube_pc, cube_ed)
            if distance < min_distance:
                min_distance = distance
                closest_cube_ed = cube_ed

        return min_distance, closest_cube_ed

    def match_with_prev(self):
        matched_cubes = []
        matched_cubes_ids = []
        unmatched_cubes = []

        prev_number_of_cubes = len(self.prev_cubes)
        for current_cube in self.matched_cubes:
            matched = False
            for prev_cube in self.prev_cubes:
                distance = self.calculate_distance(current_cube, prev_cube)

                if distance < PREV_CUBE_MATCH_DISTANCE:
                    matched = True
                    # refine cube position?
                    matched_cubes.append(Cube(prev_cube.id, current_cube.x, current_cube.y, current_cube.z, current_cube.orientation, current_cube.confidencd))
                    matched_cubes_ids.append(prev_cube.id)
                    break
            if not matched:
                unmatched_cubes.append(current_cube)

        missing_ids = self.find_missing_ids(matched_cubes_ids, prev_number_of_cubes)

        if (missing_ids):
            missing_cubes_with_id = self.match_unmatched_cubes_to_id(unmatched_cubes, missing_ids, prev_number_of_cubes)

        matched_cubes.append(cube for cube in missing_cubes_with_id)


    def find_missing_ids(self, matched_cubes_ids, prev_number_of_cubes):
        # sets of expected and matched ids
        expected_ids = set(range(0, prev_number_of_cubes))
        matched_ids = set(matched_cubes_ids)

        # find difference between expected and matched
        missing_ids = expected_ids - matched_ids

        return sorted(missing_ids)

    def match_unmatched_cubes_to_id(self, unmatched_cubes, missing_ids, prev_num_of_cubes):
        cubes = []
        id_count = 0
        for unmachted_cube in unmatched_cubes:
            if (id_count < len(missing_ids)):
                cubes.append(Cube(missing_ids[id_count], unmachted_cube.x, unmachted_cube.y, unmachted_cube.z, unmachted_cube.orientation, unmachted_cube.confidence))
                id_count += 1
            else:
                cubes.append(Cube(prev_num_of_cubes, unmachted_cube.x, unmachted_cube.y, unmachted_cube.z, unmachted_cube.orientation, unmachted_cube.confidence))
                prev_num_of_cubes += 1

        return cubes

    def calculate_distance(self, cube1, cube2):
        # Euclidean distance between two cubes
        return ((cube1.x - cube2.x) ** 2 + (cube1.y - cube2.y) ** 2)

    def publish_cubes(self):
        for cube in self.matched_cubes:
            print("Cube ",cube.id,": [",round(cube.x,3), ", ", round(cube.y,3), ", ", round(cube.z,3), ", ", cube.orientation, "]")
            cube_odom = Odometry()
            cube_odom.header.frame_id = "world"
            cube_odom.child_frame_id = "cube_{}".format(cube.id)
            cube_odom.pose.pose.position.x = cube.x
            cube_odom.pose.pose.position.y = cube.y
            cube_odom.pose.pose.position.z = cube.z
            cube_odom.pose.pose.orientation.x = 0
            cube_odom.pose.pose.orientation.y = 0
            cube_odom.pose.pose.orientation.z = self.convert_rotation(cube.orientation)
            cube_odom.pose.pose.orientation.w = 0
            #cube_odom.pose.pose.orientation.w = cube.confidence

            #print("Publishing Cube {}: ({}, {}, {}) - {}".format(cube.id, round(cube.x, 3), round(cube.y, 3), round(cube.z, 3), round(cube.orientation, 3)))
            cube_publisher = rospy.Publisher("cube_{}_odom".format(cube.id), Odometry, queue_size=10)
            cube_publisher.publish(cube_odom)

        self.matched_cubes.clear()

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
