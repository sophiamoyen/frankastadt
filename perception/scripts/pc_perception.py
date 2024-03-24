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
        self.using_icp = True
        self.single_perception = False
        self.first_perception_done = False

        # Additional parameters
        self.cube_diagonal = 0.0389
        self.edge_len = 0.045
        self.number_of_cubes = 30

        # Create a cube for ground truth
        self.cube_gt = create_cube_gt(self.edge_len)
        self.pyramid_odom = None
        self.pyramid_state = 0
        #self.cube_gt = create_pyramid_gt(3, 5, 0.045)
        #self.cube_gt.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        self.pyramid_subscriber = "/pyramid_odom"
        self.pyramid_state_subscriber = "/pyramid_state"

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
            self.boundZ = [-0.1, 0.3]
            #self.boundZ = [-0.1, 0.055]
            #self.boundZ = [0.07, 0.1]
            #self.boundZ = [0.1, 0.15]
            self.eps = 0.03
            self.min_points = 40
            self.voxel_size = 0.005
            self.icp_min_points = 50
            self.distance_threshold = 0.02

        # Create a publisher for the downsampled point cloud
        self.pub = rospy.Publisher('segmented_pc', PointCloud2, queue_size=10)

        self.num_of_cubes_pub = rospy.Publisher('num_cubes', String, queue_size=10)

        # Create a publisher for the odometry of the cubes
        self.cube_publisher = np.array([None]*self.number_of_cubes)
        for i in range(self.number_of_cubes):
            self.cube_publisher[i] = rospy.Publisher('cube_{}_odom_pc'.format(i), Odometry, queue_size=10)
    
    def publish_odometry(self, pos, rot, frame_id, child_frame_id, index):
        '''
        Publish the odometry of the cubes
        Args:
            pos (list): position of the cube
            rot (list): orientation of the cube
            frame_id (string): frame id of the odometry
            child_frame_id (string): child frame id of the odometry
            index (int): index of the cube
        '''

        cube_odom = Odometry()
        cube_odom.header.frame_id = frame_id
        cube_odom.child_frame_id = "cube_{}".format(index)
        cube_odom.pose.pose.position.x = pos[0]
        cube_odom.pose.pose.position.y = pos[1]
        cube_odom.pose.pose.position.z = pos[2]
        cube_odom.pose.pose.orientation.x = rot[0]
        cube_odom.pose.pose.orientation.y = rot[1]
        cube_odom.pose.pose.orientation.z = rot[2]
        cube_odom.pose.pose.orientation.w = rot[3]
        self.cube_publisher[index].publish(cube_odom)
    
    def pyramid_callback(self, msg):
        self.pyramid_odom = msg.pose.pose.position
    
    def pyramid_state_callback(self, msg):
        self.pyramid_state = int(msg.data)
        
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

        #downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        # Cropping
        zf_cloud = crop_pointcloud(downpcd, self.boundX, self.boundY, self.boundZ)
        
        if self.boundZ[0] <= 0:
            # Segment the largest planar component from the cropped cloud
            outlier_cloud = segment_pc(zf_cloud, self.distance_threshold)
            
            if outlier_cloud.is_empty():
                return
        else:
            outlier_cloud = zf_cloud

        # Clustering
        labels = cluster_pc(outlier_cloud, self.eps, self.min_points)
        max_label = labels.max()

        cube_count = 0

        # Determine center and rotation of each cluster
        for i in range(max_label + 1):
            #print(f"Cluster {i}: {np.count_nonzero(labels == i)} points")
            cube = outlier_cloud.select_by_index(np.where(labels == i)[0])
            if self.pyramid_odom is not None:
                if np.abs(cube.get_center()[0] - self.pyramid_odom.x) < 0.0225 and np.abs(cube.get_center()[1] - self.pyramid_odom.y) < 0.1:
                    continue

            '''
            if not self.using_icp or np.asarray(cube.points).shape[0] < self.icp_min_points:
                # Obtain the orientation of the cube
                rotation = obtain_pc_rotation(cube)
                quat = rotation_matrix_to_quaternion(rotation)

                # Obtain the position of the cube
                bounding_box = cube.get_axis_aligned_bounding_box()
                box_points = np.asarray(bounding_box.get_box_points())
                z = sum(np.sort(box_points[:, 2])[4:])/4 - self.edge_len/2
                center = bounding_box.get_center()
                center[2] = z

                # Publish the odometry of each cube  
                self.publish_odometry(center, quat, self.world_frame, self.world_frame, cube_count)
                cube_count += 1
                print("Cube " + str(cube_count) + ":")
                print(center)
                print(rotation_matrix_to_euler_angles(rotation), "\n")
                self.first_perception_done = True
            '''

            # Perform ICP to separate the cubes within the cluster
            if self.using_icp:
                max_iter = 10
                count = 0
                while np.asarray(cube.points).shape[0] > self.icp_min_points and count < max_iter:
                    count += 1

                    # ICP
                    reg_p2p = o3d.pipelines.registration.registration_icp(
                        self.cube_gt, cube, 1, np.array([[1,0,0,0.5],[0,1,0,0],[0,0,1,0.8],[0,0,0,1]]), o3d.pipelines.registration.TransformationEstimationPointToPoint())

                    
                    #if reg_p2p.inlier_rmse > 0.02 or reg_p2p.inlier_rmse == 0:
                    #    continue
                    

                    #print(reg_p2p)

                    # Retrieve position and orientation from the transformation matrix TODO: Check transformation between raw_pos and pos
                    raw_pos = [reg_p2p.transformation[0, 3], reg_p2p.transformation[1, 3], reg_p2p.transformation[2, 3]]
                    pos = [reg_p2p.transformation[1, 3], -reg_p2p.transformation[0, 3], reg_p2p.transformation[2, 3]] # Transforms pos from o3d frame to world frame
                    rotation = np.array([[reg_p2p.transformation[0, 0], -reg_p2p.transformation[1, 0], reg_p2p.transformation[2, 0]],
                                        [reg_p2p.transformation[0, 1], -reg_p2p.transformation[1, 1], reg_p2p.transformation[2, 1]],
                                        [reg_p2p.transformation[0, 2], -reg_p2p.transformation[1, 2], reg_p2p.transformation[2, 2]]])
                    rot = rotation_matrix_to_quaternion(rotation)
                    child_frame = 'cube_{}_odom'.format(cube_count)
                    self.publish_odometry(raw_pos, rot, self.world_frame, self.world_frame, cube_count)
                    cube_count += 1

                    # Remove the points within the radius of its diagonal
                    cube = cube.select_by_index(np.where(np.linalg.norm(np.asarray(cube.points) - raw_pos, axis=1) > self.cube_diagonal+0.03)[0])

                    # Print the position and orientation
                    
                    print("Cube " + str(cube_count) + ":")
                    print(raw_pos)
                    print(rotation_matrix_to_euler_angles(rotation), "\n")
                    
                    self.first_perception_done = True
        
        # Assign colors to the segmented point clouds for the visualization
        self.num_of_cubes_pub.publish(str(cube_count))
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
        pcd_ros = o3dpc_to_rospc(outlier_cloud, msg.header, msg.fields, frame_id=self.world_frame)

        self.pub.publish(pcd_ros)
    

if __name__ == '__main__':

    rospy.init_node('pc_perception_node')
    print("started pointcloud perception node")

    pc_perception = PCPerception()

    #pcd = create_pyramid_gt(5, 8, 0.045)
    """
    pcd = create_cube_gt_robot(0.045)
    
    o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[0, 0, 0],
                                  up=[-0.0694, -0.9768, 0.2024])
    """
    rospy.Subscriber(pc_perception.subscriber_node, PointCloud2, pc_perception.pointcloud_callback)
    rospy.Subscriber(pc_perception.pyramid_subscriber, Odometry, pc_perception.pyramid_callback)
    rospy.Subscriber(pc_perception.pyramid_state_subscriber, String, pc_perception.pyramid_state_callback)

    rospy.spin()