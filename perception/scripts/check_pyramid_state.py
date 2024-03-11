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
import tf2_geometry_msgs
import pcl
import open3d as o3d
import matplotlib.pyplot as plt
from pc_helper import *
from pick_and_place_module.move_functions import PlanAndMove


class PCPyramidChecking():

    def __init__(self):

        # Set the work environment
        self.work_environment = "gazebo"

        # Additional parameters
        self.cube_diagonal = 0.0389
        self.edge_len = 0.045
        self.number_of_cubes = 30

        # Detection sequence parameter
        self.iteration = 0

        self.evaluation_pc = o3d.geometry.PointCloud()
        
        # Set the parameter for simulation or real world
        if self.work_environment == "gazebo":
            self.world_frame = "world"
            self.subscriber_node = "/zed2/point_cloud/cloud_registered"
            self.boundX = [-1, 1]
            self.boundY = [-1, 1]
            self.boundZ = [-1, 1]
            self.eps = 0.015
            self.min_points = 20
            self.voxel_size = 0.001
            self.icp_min_points = 100
            self.distance_threshold = 0.01
        elif self.work_environment == "real":
            self.world_frame = "world"
            self.subscriber_node = "/zed2/zed_node/point_cloud/cloud_registered"
            self.boundX = [0, 1]
            self.boundY = [-0.5, 0.5]
            self.boundZ = [-0.1, 0.2]
            self.eps = 0.03
            self.min_points = 40
            self.voxel_size = 0.005
            self.icp_min_points = 50
            self.distance_threshold = 0.02

        # Create a publisher for the downsampled point cloud
        self.pub = rospy.Publisher('segmented_pc', PointCloud2, queue_size=10)

        # Create a publisher for the odometry of the cubes
        self.pyramid_publisher = rospy.Publisher('pyramid_odom', Odometry, queue_size=10)
        
    
    def publish_odometry(self, pos, rot, frame_id, child_frame_id):
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
        pyramid_odom.child_frame_id = "pyramid"
        pyramid_odom.pose.pose.position.x = pos[0]
        pyramid_odom.pose.pose.position.y = pos[1]
        pyramid_odom.pose.pose.position.z = pos[2]
        pyramid_odom.pose.pose.orientation.x = rot[0]
        pyramid_odom.pose.pose.orientation.y = rot[1]
        pyramid_odom.pose.pose.orientation.z = rot[2]
        pyramid_odom.pose.pose.orientation.w = rot[3]
        self.pyramid_publisher.publish(pyramid_odom)

    # Downsampling, filtering, segmentation and clustering http://www.open3d.org/docs/latest/tutorial/Basic/pointcloud.html
    def pointcloud_callback(self, msg):
        '''
        Callback function for the point cloud subscriber
        Args:
            msg (sensor_msgs.msg.PointCloud2): point cloud message
        '''
        self.iteration += 1
        #plan_and_move = PlanAndMove()
        if self.iteration == 1:
            self.iteration = 1
            # Go to standard pose
        
        elif self.iteration == 3:
            self.iteration = 3
            # Go somewhere left from standard pose

        elif self.iteration == 5:
            self.iteration = 5
            # Go somewhere right from standard pose


        else:
            # Create the ground truth pyramid
            basis = 4
            num_cubes = 6
            pyramid_gt = create_pyramid_gt(basis, num_cubes, self.edge_len)

            # Call the transformation to world frame
            transformed_pc = transform_pointcloud(msg, self.world_frame)
            if transformed_pc is None:
                return

            # Voxel downsampling
            downpcd = transformed_pc.voxel_down_sample(self.voxel_size)

            # Cropping
            zf_cloud = crop_pointcloud(downpcd, self.boundX, self.boundY, self.boundZ)

            if self.iteration < 6:
                self.evaluation_pc += zf_cloud
            else:
                # Segment the largest planar component from the cropped cloud
                outlier_cloud = segment_pc(self.evaluation_pc, self.distance_threshold)
                
                if outlier_cloud.is_empty():
                    print("No cubes found")
                    return

                # Clustering
                labels = cluster_pc(outlier_cloud, self.eps, self.min_points)
                max_label = labels.max()

                # Determine center and rotation of each cluster
                for i in range(max_label + 1):
                    cube = outlier_cloud.select_by_index(np.where(labels == i)[0])

                    # Perform ICP to separate the cubes within the cluster
                    max_iter = 10
                    count = 0
                    while np.asarray(cube.points).shape[0] > self.icp_min_points and count < max_iter:
                        count += 1

                        # ICP
                        reg_p2p = o3d.pipelines.registration.registration_icp(
                            pyramid_gt[2], cube, 1, np.array([[1,0,0,0.5],[0,1,0,0],[0,0,1,0.8],[0,0,0,1]]), o3d.pipelines.registration.TransformationEstimationPointToPoint())
                        first_config = True
                        if reg_p2p.inlier_rmse > 0.02:
                            continue
                        
                        if first_config:
                            print("Pyramid state with base=" + str(basis) + " and number of cubes=" + str(num_cubes) + " found with ICP with first gt model")
                        else: 
                            print("Pyramid state with base=" + str(basis) + " and number of cubes=" + str(num_cubes) + " found with ICP with second gt model")
                        print(reg_p2p)

                        # Retrieve position and orientation from the transformation matrix TODO: Check transformation between raw_pos and pos
                        pos = [reg_p2p.transformation[0, 3], reg_p2p.transformation[1, 3], reg_p2p.transformation[2, 3]]
                        rotation = np.array([[reg_p2p.transformation[0, 0], -reg_p2p.transformation[1, 0], reg_p2p.transformation[2, 0]],
                                            [reg_p2p.transformation[0, 1], -reg_p2p.transformation[1, 1], reg_p2p.transformation[2, 1]],
                                            [reg_p2p.transformation[0, 2], -reg_p2p.transformation[1, 2], reg_p2p.transformation[2, 2]]])
                        rot = rotation_matrix_to_quaternion(rotation)
                        self.publish_odometry(pos, rot, self.world_frame, self.world_frame)

                        # Remove the points within the radius of its diagonal
                        cube = cube.select_by_index(np.where(np.linalg.norm(np.asarray(cube.points) - pos, axis=1) > self.cube_diagonal)[0])

                        # Print the position and orientation
                        print("Position of found pyramid:")
                        print(pos)
                        print(rotation_matrix_to_euler_angles(rotation), "\n")
                        break
    
if __name__ == '__main__':

    rospy.init_node('pc_pyramid_checking_node')
    print("started pc pyramid checking node")

    pcds = create_pyramid_gt(5, 8, 0.045)
    pcd = pcds[2]
    
    o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[0, 0, 0],
                                  up=[-0.0694, -0.9768, 0.2024])
    
    pc_pyramid_checking = PCPyramidChecking()
    rospy.Subscriber(pc_pyramid_checking.subscriber_node, PointCloud2, pc_pyramid_checking.pointcloud_callback)

    rospy.spin()