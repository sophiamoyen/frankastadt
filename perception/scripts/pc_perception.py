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
<<<<<<< HEAD
=======
from pc_helper import *

>>>>>>> pc_based_grasp

class PCPerception():

    def __init__(self):

        # Set the work environment
<<<<<<< HEAD
        self.work_environment = "real"
        self.using_icp = False

        # Create a cube for ground truth
        mesh = o3d.geometry.TriangleMesh.create_box(width=0.045, height=0.045, depth=0.045)
        self.cube_gt = mesh.sample_points_uniformly(number_of_points=500)
        points = np.asarray(self.cube_gt.points)
        transformed_points = points - np.array([0.0225, 0.0225, 0.0225])
        self.cube_gt.points = o3d.utility.Vector3dVector(transformed_points)

        # Additional parameters
        self.cube_diagonal = 0.0389
=======
        self.work_environment = "gazebo"
        self.using_icp = True
        self.single_perception = False
        self.first_perception_done = False

        # Additional parameters
        self.cube_diagonal = 0.0389
        self.edge_len = 0.045
        self.number_of_cubes = 30

        # Create a cube for ground truth
        self.cube_gt = create_cube_gt(self.edge_len)
>>>>>>> pc_based_grasp

        # Set the parameter for simulation or real world
        if self.work_environment == "gazebo":
            self.world_frame = "world"
            self.subscriber_node = "/zed2/point_cloud/cloud_registered"
            self.boundX = [-1, 1]
            self.boundY = [-1, 1]
            self.boundZ = [-1, 1]
            self.eps = 0.015
            self.min_points = 20
            self.voxel_size = 0.005
            self.icp_min_points = 100
            self.distance_threshold = 0.01
        elif self.work_environment == "real":
<<<<<<< HEAD
            self.world_frame = "world"
            self.subscriber_node = "/zed2/zed_node/point_cloud/cloud_registered"
            self.boundX = [-1, 1]
            self.boundY = [-1, 1]
            self.boundZ = [-0.5, 1]
=======
            self.world_frame = "map"
            self.subscriber_node = "/zed2/zed_node/point_cloud/cloud_registered"
            self.boundX = [-0.25, 0.25]
            self.boundY = [-0.5, 0.5]
            self.boundZ = [-1, -0.2]
>>>>>>> pc_based_grasp
            self.eps = 0.03
            self.min_points = 40
            self.voxel_size = 0.005
            self.icp_min_points = 50
            self.distance_threshold = 0.02

<<<<<<< HEAD

    # Copied from open3d-ros-helper method and modified https://pypi.org/project/open3d-ros-helper/#files
    def o3dpc_to_rospc(self, o3dpc, header, fields, frame_id=None, stamp=None):
        """ convert open3d point cloud to ros point cloud
        Args:
            o3dpc (open3d.geometry.PointCloud): open3d point cloud
            frame_id (string): frame id of ros point cloud header
            stamp (rospy.Time): time stamp of ros point cloud header
        Returns:
            rospc (sensor.msg.PointCloud2): ros point cloud message
        """

        cloud_npy = np.asarray(copy.deepcopy(o3dpc.points))
        is_color = o3dpc.colors
            
        n_points = len(cloud_npy[:, 0])
        if is_color:
            data = np.zeros(n_points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)
            ])
        else:
            data = np.zeros(n_points, dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32)
                ])
        data['x'] = cloud_npy[:, 0]
        data['y'] = cloud_npy[:, 1]
        data['z'] = cloud_npy[:, 2]
        
        if is_color:
            rgb_npy = np.asarray(copy.deepcopy(o3dpc.colors))
            rgb_npy = np.floor(rgb_npy*255) # nx3 matrix
            rgb_npy = rgb_npy[:, 0] * 2**16 + rgb_npy[:, 1] * 2**8 + rgb_npy[:, 2]  
            rgb_npy = rgb_npy.astype(np.uint32)
            data['rgb'] = rgb_npy

        rospc = pc2.create_cloud(header, fields, data)
        if frame_id is not None:
            rospc.header.frame_id = frame_id
        return rospc

    def rotation_matrix_to_euler_angles(self, R):
        # Extract pitch (around y-axis, arcsin of the element at (2,0))
        pitch = np.arcsin(-R[2, 0])

        # Calculate the other angles based on pitch
        if np.abs(np.cos(pitch)) > 1e-6:
            # Not at poles
            yaw = np.arctan2(R[1, 0], R[0, 0])
            roll = np.arctan2(R[2, 1], R[2, 2])
        else:
            # At poles
            yaw = 0
            roll = np.arctan2(-R[0, 1], R[1, 1])

        return np.array([roll, pitch, yaw])


    def rotation_matrix_to_quaternion(self, R):
        """
        Convert a 3x3 rotation matrix to its quaternion representation.

        Args:
        - R: 3x3 rotation matrix

        Returns:
        - Quaternion [x, y, z, w]
        """
        q = np.empty(4)

        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s

        return q

    def transform_pointcloud(self, msg, target_frame):

        # Get the transform from the camera frame to the world frame
        try: 
            transform = tf_buffer.lookup_transform(target_frame,
                                                msg.header.frame_id,
                                                rospy.Time(0),
                                                rospy.Duration(2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

        # Conversion to open3d point cloud
        msg.header.frame_id = target_frame
        o3d_pc = o3d.geometry.PointCloud()
        points = pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True)
        o3d_pc.points = o3d.utility.Vector3dVector(points)

        # Transform the point cloud
        translation = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
        rotation = np.array([transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z])
        rotation_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(rotation)
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation
        o3d_pc.transform(transformation_matrix)
        
        return o3d_pc

    def obtain_pc_rotation(self, pc):
        
        # Save volume of axis aligned bounding box of pc
        bounding_box = pc.get_axis_aligned_bounding_box()
        volume = bounding_box.volume()

        # Rotate the point cloud by 1/32 of a circle and save the rotation that results in the smallest volume
        rotation = 0
        for i in range(0, 32):
            pc.rotate(pc.get_rotation_matrix_from_xyz((0, 0, np.pi/64)), center=(0, 0, 0))
            bounding_box = pc.get_axis_aligned_bounding_box()
            new_volume = bounding_box.volume()
            if new_volume < volume:
                volume = new_volume
                rotation = (i+1)*np.pi/64
        
        return pc.get_rotation_matrix_from_xyz((0, 0, rotation))

    # Downsampling, filtering, segmentation and clustering http://www.open3d.org/docs/latest/tutorial/Basic/pointcloud.html
    def pointcloud_callback(self, msg):
        
        # Call the transformation to world frame
        transformed_pc = self.transform_pointcloud(msg, self.world_frame)
        if transformed_pc is None:
            return None

        # Voxel downsampling
        downpcd = transformed_pc.voxel_down_sample(self.voxel_size)

        # Crop the point cloud to the area of interest
        xf_cloud = downpcd.crop(
            o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(self.boundX[0], -np.inf, -np.inf),
            max_bound=(self.boundX[1], np.inf, np.inf)
            )
        )
        
        yf_cloud = xf_cloud.crop(
            o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(-np.inf, self.boundY[0], -np.inf),
            max_bound=(np.inf, self.boundY[1], np.inf)
            )
        )
        
        zf_cloud = yf_cloud.crop(
            o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(-np.inf, -np.inf, self.boundZ[0]),
            max_bound=(np.inf, np.inf, self.boundZ[1])
            )
        )

        # Segment the largest planar component from the cropped cloud
        plane_model, inliers = zf_cloud.segment_plane(distance_threshold=self.distance_threshold,
                                                        ransac_n=3,
                                                        num_iterations=1000)
        [a, b, c, d] = plane_model
        outlier_cloud = zf_cloud.select_by_index(inliers, invert=True)
        
        # Clustering
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
                labels = np.array(
                    outlier_cloud.cluster_dbscan(eps=self.eps, min_points=self.min_points, print_progress=True))
        max_label = labels.max()

        # Determine center and rotation of each cluster
        for i in range(max_label + 1):
            print(f"Cluster {i}: {np.count_nonzero(labels == i)} points")
            cube = outlier_cloud.select_by_index(np.where(labels == i)[0])
            bounding_box = cube.get_axis_aligned_bounding_box()
            rotation = self.obtain_pc_rotation(cube)
            quat = self.rotation_matrix_to_quaternion(rotation)
            box_points = np.asarray(bounding_box.get_box_points())
            edge_len = 0.0225
            z = sum(np.sort(box_points[:, 2])[4:])/4 - edge_len
            center = bounding_box.get_center()
            center[2] = z
=======
        # Create a publisher for the downsampled point cloud
        self.pub = rospy.Publisher('segmented_pc', PointCloud2, queue_size=10)

        # Create a publisher for the odometry of the cubes
        self.cube_publisher = np.array([None]*self.number_of_cubes)
        for i in range(self.number_of_cubes):
            self.cube_publisher[i] = rospy.Publisher('cube_'+str(i), Odometry, queue_size=10)
    
    #def perform_icp(self, source, target):
        
    
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
        cube_odom.child_frame_id = child_frame_id
        cube_odom.pose.pose.position.x = pos[0]
        cube_odom.pose.pose.position.y = pos[1]
        cube_odom.pose.pose.position.z = pos[2]
        cube_odom.pose.pose.orientation.x = rot[0]
        cube_odom.pose.pose.orientation.y = rot[1]
        cube_odom.pose.pose.orientation.z = rot[2]
        cube_odom.pose.pose.orientation.w = rot[3]
        self.cube_publisher[index].publish(cube_odom)

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

        if self.single_perception and self.first_perception_done:
            return

        # Voxel downsampling
        downpcd = transformed_pc.voxel_down_sample(self.voxel_size)

        # Cropping
        zf_cloud = crop_pointcloud(downpcd, self.boundX, self.boundY, self.boundZ)

        # Segment the largest planar component from the cropped cloud
        outlier_cloud = segment_pc(zf_cloud, self.distance_threshold)
        
        if outlier_cloud.is_empty():
            return

        # Clustering
        labels = cluster_pc(outlier_cloud, self.eps, self.min_points)
        max_label = labels.max()

        cube_count = 0

        # Determine center and rotation of each cluster
        for i in range(max_label + 1):
            #print(f"Cluster {i}: {np.count_nonzero(labels == i)} points")
            cube = outlier_cloud.select_by_index(np.where(labels == i)[0])

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
>>>>>>> pc_based_grasp

            # Perform ICP to separate the cubes within the cluster
            if self.using_icp:
                max_iter = 10
                count = 0
                while np.asarray(cube.points).shape[0] > self.icp_min_points and count < max_iter:
                    count += 1

                    # ICP
                    reg_p2p = o3d.pipelines.registration.registration_icp(
                        self.cube_gt, cube, 1, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                    )

<<<<<<< HEAD
                    # Retrieve position and orientation from the transformation matrix
=======
                    # Retrieve position and orientation from the transformation matrix TODO: Check transformation between raw_pos and pos
>>>>>>> pc_based_grasp
                    raw_pos = [reg_p2p.transformation[0, 3], reg_p2p.transformation[1, 3], reg_p2p.transformation[2, 3]]
                    pos = [reg_p2p.transformation[1, 3], -reg_p2p.transformation[0, 3], reg_p2p.transformation[2, 3]] # Transforms pos from o3d frame to world frame
                    rotation = np.array([[reg_p2p.transformation[0, 0], -reg_p2p.transformation[1, 0], reg_p2p.transformation[2, 0]],
                                        [reg_p2p.transformation[0, 1], -reg_p2p.transformation[1, 1], reg_p2p.transformation[2, 1]],
                                        [reg_p2p.transformation[0, 2], -reg_p2p.transformation[1, 2], reg_p2p.transformation[2, 2]]])
<<<<<<< HEAD
                    rot = self.rotation_matrix_to_quaternion(rotation)
=======
                    rot = rotation_matrix_to_quaternion(rotation)

                    self.publish_odometry(pos, rot, self.world_frame, self.world_frame, cube_count)
                    cube_count += 1
>>>>>>> pc_based_grasp

                    # Remove the points within the radious of its diagonal
                    cube = cube.select_by_index(np.where(np.linalg.norm(np.asarray(cube.points) - raw_pos, axis=1) > self.cube_diagonal)[0])

                    # Print the position and orientation
<<<<<<< HEAD
                    print(pos)
                    print(self.rotation_matrix_to_euler_angles(rotation))


            # Publish the odometry of the first cube TODO: publish all cubes   
            if i == 0 and not self.using_icp:
                cube_odom = Odometry()
                cube_odom.header.frame_id = self.world_frame
                cube_odom.child_frame_id = self.world_frame
                cube_odom.pose.pose.position.x = center[0]
                cube_odom.pose.pose.position.y = center[1]
                cube_odom.pose.pose.position.z = center[2]
                cube_odom.pose.pose.orientation.x = quat[0]
                cube_odom.pose.pose.orientation.y = quat[1]
                cube_odom.pose.pose.orientation.z = quat[2]
                cube_odom.pose.pose.orientation.w = quat[3]
                transform_pub.publish(cube_odom)
                print("published odometry: ", cube_odom)

            if i == 0 and self.using_icp:
                cube_odom = Odometry()
                cube_odom.header.frame_id = self.world_frame
                cube_odom.child_frame_id = self.world_frame
                cube_odom.pose.pose.position.x = pos[0]
                cube_odom.pose.pose.position.y = pos[1]
                cube_odom.pose.pose.position.z = pos[2]
                cube_odom.pose.pose.orientation.x = rot[0]
                cube_odom.pose.pose.orientation.y = rot[1]
                cube_odom.pose.pose.orientation.z = rot[2]
                cube_odom.pose.pose.orientation.w = rot[3]
                transform_pub.publish(cube_odom)
                print("published odometry: ", cube_odom)

            if not self.using_icp:
                print(center)
                print(self.rotation_matrix_to_euler_angles(rotation))
=======
                    print("Cube " + str(cube_count) + ":")
                    print(raw_pos)
                    print(rotation_matrix_to_euler_angles(rotation), "\n")
                    self.first_perception_done = True
>>>>>>> pc_based_grasp
        
        # Assign colors to the segmented point clouds for the visualization
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
<<<<<<< HEAD
        pcd_ros = self.o3dpc_to_rospc(outlier_cloud, msg.header, msg.fields, frame_id=self.world_frame)

        #print("publish segmented point cloud")
        pub.publish(pcd_ros)
=======
        pcd_ros = o3dpc_to_rospc(outlier_cloud, msg.header, msg.fields, frame_id=self.world_frame)

        self.pub.publish(pcd_ros)
>>>>>>> pc_based_grasp
    

if __name__ == '__main__':

    rospy.init_node('pc_perception_node')
    print("started pointcloud perception node")

<<<<<<< HEAD
    # Create a TF2 buffer and listener
    print("started node")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    pc_perception = PCPerception()

    # Create a publisher for the downsampled point cloud
    pub = rospy.Publisher('segmented_pc', PointCloud2, queue_size=10)

    # Create a publisher for the odometry of the cubes
    cube_1 = rospy.Publisher('cube_1', Odometry, queue_size=10)
    cube_2 = rospy.Publisher('cube_2', Odometry, queue_size=10)
    cube_3 = rospy.Publisher('cube_3', Odometry, queue_size=10)
    cube_4 = rospy.Publisher('cube_4', Odometry, queue_size=10)
    cube_5 = rospy.Publisher('cube_5', Odometry, queue_size=10)
    cube_6 = rospy.Publisher('cube_6', Odometry, queue_size=10)
    cube_7 = rospy.Publisher('cube_7', Odometry, queue_size=10)
    cube_8 = rospy.Publisher('cube_8', Odometry, queue_size=10)
    cube_9 = rospy.Publisher('cube_9', Odometry, queue_size=10)
    cube_10 = rospy.Publisher('cube_10', Odometry, queue_size=10)
    cube_11 = rospy.Publisher('cube_11', Odometry, queue_size=10)
    cube_12 = rospy.Publisher('cube_12', Odometry, queue_size=10)
    cube_13 = rospy.Publisher('cube_13', Odometry, queue_size=10)
    cube_14 = rospy.Publisher('cube_14', Odometry, queue_size=10)
    cube_15 = rospy.Publisher('cube_15', Odometry, queue_size=10)
    cube_16 = rospy.Publisher('cube_16', Odometry, queue_size=10)
    cube_17 = rospy.Publisher('cube_17', Odometry, queue_size=10)
    cube_18 = rospy.Publisher('cube_18', Odometry, queue_size=10)
    cube_19 = rospy.Publisher('cube_19', Odometry, queue_size=10)
    cube_20 = rospy.Publisher('cube_20', Odometry, queue_size=10)
    cube_21 = rospy.Publisher('cube_21', Odometry, queue_size=10)
    cube_22 = rospy.Publisher('cube_22', Odometry, queue_size=10)
    cube_23 = rospy.Publisher('cube_23', Odometry, queue_size=10)
    cube_24 = rospy.Publisher('cube_24', Odometry, queue_size=10)
    cube_25 = rospy.Publisher('cube_25', Odometry, queue_size=10)
    cube_26 = rospy.Publisher('cube_26', Odometry, queue_size=10)
    cube_27 = rospy.Publisher('cube_27', Odometry, queue_size=10)
    cube_28 = rospy.Publisher('cube_28', Odometry, queue_size=10)
    cube_29 = rospy.Publisher('cube_29', Odometry, queue_size=10)
    cube_30 = rospy.Publisher('cube_30', Odometry, queue_size=10)


    transform_pub = rospy.Publisher('cube', Odometry, queue_size=10)
    # Subscribe to the pointcloud topic
=======
    pc_perception = PCPerception()

>>>>>>> pc_based_grasp
    rospy.Subscriber(pc_perception.subscriber_node, PointCloud2, pc_perception.pointcloud_callback)

    rospy.spin()