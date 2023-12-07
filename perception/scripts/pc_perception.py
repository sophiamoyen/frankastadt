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

# Copied from open3d-ros-helper method and modified https://pypi.org/project/open3d-ros-helper/#files
def o3dpc_to_rospc(o3dpc, header, fields, frame_id=None, stamp=None):
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

def rotation_matrix_to_euler_angles(R):
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


def rotation_matrix_to_quaternion(R):
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

def transform_pointcloud(msg, target_frame):

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

def obtain_pc_rotation(pc):
    
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
def pointcloud_callback(msg):
    
    # Call the transformation to world frame
    world_frame = "world"
    transformed_pc = transform_pointcloud(msg, world_frame)
    if transformed_pc is None:
        return

    # Voxel downsampling
    voxel_size = 0.01
    downpcd = transformed_pc.voxel_down_sample(voxel_size)
    xf_cloud = downpcd.crop(
        o3d.geometry.AxisAlignedBoundingBox(
        min_bound=(-1, -np.inf, -np.inf),
        max_bound=(1, np.inf, np.inf)
        )
    )
    
    yf_cloud = xf_cloud.crop(
        o3d.geometry.AxisAlignedBoundingBox(
        min_bound=(-np.inf, -1, -np.inf),
        max_bound=(np.inf, 1, np.inf)
        )
    )

    # Segment the largest planar component from the cropped cloud
    plane_model, inliers = yf_cloud.segment_plane(distance_threshold=0.01,
                                                    ransac_n=3,
                                                    num_iterations=1000)
    [a, b, c, d] = plane_model
    outlier_cloud = yf_cloud.select_by_index(inliers, invert=True)

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(
                outlier_cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

    # Get the box coordinates of the segmented cubes
    max_label = labels.max()
    cubes = []
    for i in range(max_label + 1):
        print(f"Cluster {i}: {np.count_nonzero(labels == i)} points")
        if np.count_nonzero(labels == i) > 10:
            cube = outlier_cloud.select_by_index(np.where(labels == i)[0])
            bounding_box = cube.get_axis_aligned_bounding_box()
            rotation = obtain_pc_rotation(cube)
            quat = rotation_matrix_to_quaternion(rotation)
            box_points = np.asarray(bounding_box.get_box_points())
            edge_len = 0.0225
            z = sum(np.sort(box_points[:, 2])[4:])/4 - edge_len
            center = bounding_box.get_center()
            center[2] = z

            #transformation_matrix = np.eye(4)
            #transformation_matrix[:3, :3] = rotation
            #transformation_matrix[:3, 3] = center

            if i == 0:
                cube_odom = Odometry()
                cube_odom.header.frame_id = "world"
                cube_odom.child_frame_id = "world"
                cube_odom.pose.pose.position.x = center[0]
                cube_odom.pose.pose.position.y = center[1]
                cube_odom.pose.pose.position.z = center[2]
                cube_odom.pose.pose.orientation.x = quat[0]
                cube_odom.pose.pose.orientation.y = quat[1]
                cube_odom.pose.pose.orientation.z = quat[2]
                cube_odom.pose.pose.orientation.w = quat[3]
                transform_pub.publish(cube_odom)
                print("published odometry: ", cube_odom)

            print(center)
            print(rotation_matrix_to_euler_angles(rotation))
        else:
            print("cluster too small, skipping")
    
    # Assign colors to the segmented point clouds for the visualization
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    pcd_ros = o3dpc_to_rospc(outlier_cloud, msg.header, msg.fields, frame_id=world_frame)

    #print("publish segmented point cloud")
    pub.publish(pcd_ros)
    

if __name__ == '__main__':
    rospy.init_node('pc_perception_node')

    # Create a TF2 buffer and listener
    print("started node")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Create a publisher for the downsampled point cloud
    pub = rospy.Publisher('segmented_pc', PointCloud2, queue_size=10)
    transform_pub = rospy.Publisher('cube', Odometry, queue_size=10)
    # Subscribe to the pointcloud topic
    rospy.Subscriber('/zed2/point_cloud/cloud_registered', PointCloud2, pointcloud_callback)

    rospy.spin()
