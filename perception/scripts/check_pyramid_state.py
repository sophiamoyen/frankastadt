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
        self.work_environment = "real"
        self.plan_and_move = PlanAndMove()
        self.sleep_turn = False


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
            self.boundY = [-0.75, 0.75]
            self.boundZ = [-0.1, 0.2]
            self.eps = 0.03
            self.min_points = 40
            self.voxel_size = 0.005
            self.icp_min_points = 50
            self.distance_threshold = 0.02

        # Create a publisher for the downsampled point cloud
        self.pub = rospy.Publisher('pyramid_pc', PointCloud2, queue_size=10)

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
        if self.sleep_turn:
            rospy.sleep(2)
            self.sleep_turn = False
            return
        
        if self.iteration == 0:
            # Go to standard pose
            self.iteration += 1
            self.plan_and_move.move_standard_pose()
            self.sleep_turn = True
        
        elif self.iteration == 2:
            # Go somewhere left from standard pose
            self.iteration += 1
            self.plan_and_move.move_left_pose()
            self.sleep_turn = True
        
        elif self.iteration == 4:
            # Go somewhere right from standard pose
            self.iteration += 1
            self.plan_and_move.move_right_pose()
            self.sleep_turn = True
        

        elif self.iteration != -1:
            # Create the ground truth pyramid
            """
            basis = 3
            num_cubes = 5
            pyramid_gt = create_pyramid_gt(basis, num_cubes, self.edge_len)
            """

            # Call the transformation to world frame
            transformed_pc = transform_pointcloud(msg, self.world_frame)
            if transformed_pc is None:
                return

            # Voxel downsampling
            downpcd = transformed_pc.voxel_down_sample(self.voxel_size)

            # Cropping
            zf_cloud = crop_pointcloud(downpcd, self.boundX, self.boundY, self.boundZ)

            if self.iteration < 5:
                self.iteration += 1
                print("adding up pc")
                self.evaluation_pc += zf_cloud
                print("pc size:", self.evaluation_pc)

            else:
                self.evaluation_pc += zf_cloud
                print("pc size:", self.evaluation_pc)
                # Segment the largest planar component from the cropped cloud
                outlier_cloud = segment_pc(self.evaluation_pc, self.distance_threshold)
                
                if outlier_cloud.is_empty():
                    print("No cubes found")
                    return

                # Clustering
                labels = cluster_pc(outlier_cloud, self.eps, self.min_points)
                max_label = labels.max()

                # Determine center and rotation of each cluster
                for j in range(2,7):
                    pyramid_gt = create_pyramid_gt(3, j, self.edge_len)
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
                            if reg_p2p.inlier_rmse > 0.0075 or reg_p2p.inlier_rmse < 0.006:
                                continue
                            
                        
                            print("Pyramid state with base=3 and number of cubes=" + str(j) + " found with ICP")

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
                print("done searching for pyramid")
                # Assign colors to the segmented point clouds for the visualization
                colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
                colors[labels < 0] = 0
                outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
                pcd_ros = o3dpc_to_rospc(outlier_cloud, msg.header, msg.fields, frame_id=self.world_frame)
                self.iteration = -1
                self.pub.publish(pcd_ros)
                
                
    
if __name__ == '__main__':

    """
    pcds = create_pyramid_gt(5, 8, 0.045)
    pcd = pcds[2]
    
    o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[0, 0, 0],
                                  up=[-0.0694, -0.9768, 0.2024])
    """
    pc_pyramid_checking = PCPyramidChecking()
    rospy.Subscriber(pc_pyramid_checking.subscriber_node, PointCloud2, pc_pyramid_checking.pointcloud_callback)

    rospy.spin()