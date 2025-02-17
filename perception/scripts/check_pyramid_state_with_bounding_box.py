#!/usr/bin/env python3

import rospy
import copy
import tf2_ros
import struct
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_geometry_msgs
import pcl
import open3d as o3d
import matplotlib.pyplot as plt
from pc_helper import *


class PCPerception():

    def __init__(self):

        # Set the work environment
        self.work_environment = "real"

        # Additional parameters
        self.cube_diagonal = 0.0389
        self.edge_len = 0.045
        self.number_of_cubes = 30

        # Create a cube for ground truth
        self.cube_gt = create_cube_gt(self.edge_len)

        # Set the parameter for simulation or real world
        if self.work_environment == "gazebo":
            self.world_frame = "world"
            self.subscriber_node = "/zed2/point_cloud/cloud_registered"
            self.boundX = [-1, 1]
            self.boundY = [-1, 1]
            self.boundsZ = [[0, 0.84],[0.84, 0.89],[0.88, 1]]
            self.eps = 0.015
            self.min_points = 20
            self.voxel_size = 0.001
            self.icp_min_points = 10
            self.distance_threshold = 0.01
        elif self.work_environment == "real":
            self.world_frame = "world"
            self.subscriber_node = "/zed2/zed_node/point_cloud/cloud_registered"
            self.boundX = [0, 1]
            self.boundY = [-0.5, 0.5]
            self.boundsZ = [[-0.1, 0.05],[0.07, 0.1],[0.1, 0.15]]
            self.eps = 0.03
            self.min_points = 40
            self.voxel_size = 0.005
            self.icp_min_points = 50
            self.distance_threshold = 0.02

        # Create a publisher for the downsampled point cloud
        self.pub = rospy.Publisher('segmented_pc', PointCloud2, queue_size=10)

        self.num_of_cubes_pub = rospy.Publisher('pyramid_state', String, queue_size=10)

        self.pyramid_odom_pub = rospy.Publisher('pyramid_odom', Odometry, queue_size=10)
    
    def check_cube_distance(self, positions, dist):

        for i in range(len(positions)):
            for j in range(i+1, len(positions)):
                distance = np.linalg.norm(np.array(positions[i]) - np.array(positions[j]))
                if distance > dist:
                    return False
        return True
        
    
    def publish_odometry(self, pos, frame_id):
        '''
        Publish the odometry of the cubes
        Args:
            pos (list): position of the cube
            rot (list): orientation of the cube
            frame_id (string): frame id of the odometry
            child_frame_id (string): child frame id of the odometry
            index (int): index of the cube
        '''

        pyramid_odom = Odometry()
        pyramid_odom.header.frame_id = frame_id
        pyramid_odom.child_frame_id = "pyramid_odom"
        pyramid_odom.pose.pose.position.x = pos[0]
        pyramid_odom.pose.pose.position.y = pos[1]
        pyramid_odom.pose.pose.position.z = pos[2]
        self.pyramid_odom_pub.publish(pyramid_odom)

    # Downsampling, filtering, segmentation and clustering http://www.open3d.org/docs/latest/tutorial/Basic/pointcloud.html
    def pointcloud_callback(self, msg):
        '''
        Callback function for the point cloud subscriber
        Args:
            msg (sensor_msgs.msg.PointCloud2): point cloud message
        '''
        
        # Call the transformation to world frame
        transformed_pc = transform_pointcloud(msg, self.world_frame)
        if transformed_pc is None:
            return

        # Voxel downsampling
        downpcd = transformed_pc.voxel_down_sample(self.voxel_size)

        cube_count = 0
        positions = []
        for j in range(2,-1,-1):
            # Cropping
            zf_cloud = crop_pointcloud(downpcd, self.boundX, self.boundY, self.boundsZ[j])

            if zf_cloud.is_empty():
                continue

            # Segment the largest planar component from the cropped cloud
            if j == 0:
                outlier_cloud = segment_pc(zf_cloud, self.distance_threshold)
                
                if outlier_cloud.is_empty():
                    continue
            else:
                outlier_cloud = zf_cloud
            # Clustering
            labels = cluster_pc(outlier_cloud, self.eps, self.min_points)
            max_label = labels.max()
            layer = 0
            base_candidates = []
            odom = [0,0,0]
            
            # Determine center and rotation of each cluster
            for i in range(max_label + 1):
                #print(f"Cluster {i}: {np.count_nonzero(labels == i)} points")
                cube = outlier_cloud.select_by_index(np.where(labels == i)[0])
                #print(len(cube.points))

                # Perform ICP to separate the cubes within the cluster
                max_iter = 10
                count = 0
                cubes = []
                
                base_detected = False
                while np.asarray(cube.points).shape[0] > self.icp_min_points and count < max_iter:
                    count += 1
                    # ICP
                    reg_p2p = o3d.pipelines.registration.registration_icp(
                        self.cube_gt, cube, 1, np.array([[1,0,0,0.5],[0,1,0,0],[0,0,1,0.8],[0,0,0,1]]), o3d.pipelines.registration.TransformationEstimationPointToPoint())

                    # Retrieve position and orientation from the transformation matrix TODO: Check transformation between raw_pos and pos
                    pos = [reg_p2p.transformation[0, 3], reg_p2p.transformation[1, 3], reg_p2p.transformation[2, 3]]
                    rotation = np.array([[reg_p2p.transformation[0, 0], -reg_p2p.transformation[1, 0], reg_p2p.transformation[2, 0]],
                                        [reg_p2p.transformation[0, 1], -reg_p2p.transformation[1, 1], reg_p2p.transformation[2, 1]],
                                        [reg_p2p.transformation[0, 2], -reg_p2p.transformation[1, 2], reg_p2p.transformation[2, 2]]])
                    rot = rotation_matrix_to_quaternion(rotation)
                    child_frame = 'cube_{}_odom'.format(cube_count)
                    if j == 2:
                        positions.append(pos)
                        layer = 2
                        break
                    elif j == 1:
                        positions.append(pos)
                        if layer == 0:
                            layer = 1
                        if count == 2:
                            break
                    elif j == 0 and len(positions) == 0 and not base_detected:
                        print(len(cube.points))
                        if len(cube.points) > 500:
                            cube_count = 3
                            odom = np.array(cube.get_center()) + np.array([0, 0.015, 0])
                            break
                        elif len(cube.points) > 350:
                            cube_count = 2
                            odom = np.array(cube.get_center()) - np.array([0, 0.0225, 0])
                            print(odom)
                            break
                    """
                    elif j == 0 and len(positions) == 0:
                        cubes.append(pos)
                        if len(cubes) > len(base_candidates) and self.check_cube_distance(cubes, 0.06):
                            base_candidates = cubes
                        if count == 3:
                            break
                    
                    elif j == 0 and len(positions) > 0:
                        #print("H")
                        cubes.append(pos)
                        if len(cubes) > len(base_candidates) and self.check_cube_distance(cubes, 0.06) and np.linalg.norm(np.array(cubes[0]) - np.array(positions[0])) < 0.15:
                            base_candidates = cubes
                        if count == 3:
                            break
                    """
                        
                    #self.publish_odometry(pos, rot, self.world_frame, self.world_frame, cube_count)

                    # Remove the points within the radius of its diagonal
                    cube = cube.select_by_index(np.where(np.linalg.norm(np.asarray(cube.points) - pos, axis=1) > self.cube_diagonal)[0])

                    # Print the position and orientation
                    #print("Cube " + str(cube_count) + ":")
                    #print(pos)
                    #print(rotation_matrix_to_euler_angles(rotation), "\n")
        
        # Assign colors to the segmented point clouds for the visualization
            """
            if j == 0:
                for k in range(len(base_candidates)):
                    positions.append(base_candidates[k])
                cube_count += len(base_candidates)
            else:
                cube_count += count
            """
            if layer == 2:
                cube_count = 6
                self.publish_odometry(positions[0] - np.array([0, 0, 0.09]), self.world_frame)
                break
            elif layer == 1:
                cube_count = 3 + count
                if len(positions) > 1:
                    self.publish_odometry((np.asarray(positions[0])+np.asarray(positions[1]))/2, self.world_frame)
                else:
                    self.publish_odometry(positions[0]-np.array([0,0.0225,0.045]), self.world_frame)
                break
            elif layer == 0:
                #cube_count = len(base_candidates)
                """
                if len(base_candidates) > 2:
                    self.publish_odometry((np.asarray(base_candidates[0])+np.asarray(base_candidates[1])+np.asarray(base_candidates[2]))/3, self.world_frame)
                elif len(base_candidates) > 1:
                    self.publish_odometry(base_candidates[0], self.world_frame) if np.linalg.norm(np.array(base_candidates[0])) < np.linalg.norm(np.array(base_candidates[1])) else self.publish_odometry(base_candidates[1], self.world_frame)
                """
                self.publish_odometry(odom, self.world_frame)
        #colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        #colors[labels < 0] = 0
        #outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
        #pcd_ros = o3dpc_to_rospc(outlier_cloud, msg.header, msg.fields, frame_id=self.world_frame)
        #print(positions)
        self.num_of_cubes_pub.publish(str(cube_count))
        #self.pub.publish(pcd_ros)
    

if __name__ == '__main__':

    rospy.init_node('pyramid_checking_node')

    pc_perception = PCPerception()

    rospy.Subscriber(pc_perception.subscriber_node, PointCloud2, pc_perception.pointcloud_callback)

    rospy.spin()