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

def cluster_pc(pc, eps, min_points):
    '''
    Cluster the point cloud
    Args:
        pc (open3d.geometry.PointCloud): point cloud
        eps (float): maximum distance between two points to be considered as in the same neighborhood
        min_points (int): minimum number of points to form a cluster
    Returns:
        labels (np.array): array of cluster labels
    '''

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(
                pc.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
    return labels

def create_cube_gt(edge_len):
    '''
    Create a cube with the given edge length
    Args:
        edge_len (float): edge length of the cube
    Returns:
        cube_gt (open3d.geometry.PointCloud): point cloud of the cube
    '''
    
    mesh = o3d.geometry.TriangleMesh.create_box(width=edge_len, height=edge_len, depth=edge_len)
    cube_gt = mesh.sample_points_uniformly(number_of_points=5000)
    cube_gt_x = cube_gt.select_by_index(np.where(np.asarray(cube_gt.points)[:, 0] < 0.001)[0])
    cube_gt_y = cube_gt.select_by_index(np.where(np.asarray(cube_gt.points)[:, 1] < 0.001)[0])
    cube_gt_z = cube_gt.select_by_index(np.where(np.asarray(cube_gt.points)[:, 2] > edge_len - 0.001)[0])
    cube_gt = cube_gt_x + cube_gt_y + cube_gt_z
    points = np.asarray(cube_gt.points)
    transformed_points = points - np.array([edge_len/2, edge_len/2, edge_len/2])
    cube_gt.points = o3d.utility.Vector3dVector(transformed_points)
    return cube_gt

def create_pyramid_gt(base_len, num_cubes, edge_len):
    '''
    Create a pyramid with the given base length and number of cubes
    Args:
        base_len (float): base length of the pyramid
        num_cubes (int): number of cubes
    Returns:
        pyramid_gt (open3d.geometry.PointCloud): point cloud of the pyramid
    '''
    pyramid_gt = o3d.geometry.PointCloud()
    counter = 0
    for i in range(base_len, 0, -1):
        for j in range(i):
            counter += 1
            cube = create_cube_gt(edge_len)
            cube.points = o3d.utility.Vector3dVector(np.asarray(cube.points) + np.array([j*edge_len +(base_len-i)*(edge_len/2), 0, (base_len-i)*edge_len]))
            pyramid_gt += cube
            if counter >= num_cubes:
                pyramid_gt_x = pyramid_gt.select_by_index(np.where(np.asarray(pyramid_gt.points)[:, 0] < 0.001+(base_len-i)*edge_len/2)[0])
                pyramid_gt_y = pyramid_gt.select_by_index(np.where(np.asarray(pyramid_gt.points)[:, 1] < 0.001-edge_len/2)[0])
                pyramid_gt_z = pyramid_gt.select_by_index(np.where(np.asarray(pyramid_gt.points)[:, 2] > (2*(base_len-i)+1)*edge_len/2 - 0.001)[0])
                pyramid_gt = pyramid_gt_x + pyramid_gt_y + pyramid_gt_z
                return pyramid_gt
        pyramid_gt_x = pyramid_gt.select_by_index(np.where(np.asarray(pyramid_gt.points)[:, 0] < 0.001+(base_len-i)*edge_len/2)[0])
        pyramid_gt_y = pyramid_gt.select_by_index(np.where(np.asarray(pyramid_gt.points)[:, 1] < 0.001-edge_len/2)[0])
        pyramid_gt_z = pyramid_gt.select_by_index(np.where(np.asarray(pyramid_gt.points)[:, 2] > (2*(base_len-i)+1)*edge_len - 0.001)[0])
        pyramid_gt = pyramid_gt_x + pyramid_gt_y + pyramid_gt_z
    return pyramid_gt

def crop_pointcloud(pc, boundX, boundY, boundZ):
    '''
    Crop the point cloud to the given bounds
    Args:
        pc (open3d.geometry.PointCloud): point cloud
        boundX (list): x-axis bounds
        boundY (list): y-axis bounds
        boundZ (list): z-axis bounds
    Returns:
        zf_cloud (open3d.geometry.PointCloud): cropped point cloud
    '''

    xf_cloud = pc.crop(
        o3d.geometry.AxisAlignedBoundingBox(
        min_bound=(boundX[0], -np.inf, -np.inf),
        max_bound=(boundX[1], np.inf, np.inf)
        )
    )
    
    yf_cloud = xf_cloud.crop(
        o3d.geometry.AxisAlignedBoundingBox(
        min_bound=(-np.inf, boundY[0], -np.inf),
        max_bound=(np.inf, boundY[1], np.inf)
        )
    )
    
    zf_cloud = yf_cloud.crop(
        o3d.geometry.AxisAlignedBoundingBox(
        min_bound=(-np.inf, -np.inf, boundZ[0]),
        max_bound=(np.inf, np.inf, boundZ[1])
        )
    )
    return zf_cloud

# Copied from open3d-ros-helper method and modified https://pypi.org/project/open3d-ros-helper/#files
def o3dpc_to_rospc(o3dpc, header, fields, frame_id=None, stamp=None):
    """ 
    Convert open3d point cloud to ros point cloud
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
    
def obtain_pc_rotation(pc):
    '''
    Obtain the rotation of the point cloud
    Args:
        pc (open3d.geometry.PointCloud): point cloud
    Returns:
        rotation (np.array): rotation matrix
    '''
        
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

def rotation_matrix_to_euler_angles(R):
    '''
    Convert a 3x3 rotation matrix to its euler angles representation.
    Args:
        R (np.array): 3x3 rotation matrix
    Returns:
        euler_angles (np.array): euler angles
    '''

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
        R (np.array): 3x3 rotation matrix
    Returns:
        q (np.array): quaternions
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

def segment_pc(pc, distance_threshold):
    '''
    Segment the largest planar component from the point cloud
    Args:
        pc (open3d.geometry.PointCloud): point cloud
        distance_threshold (float): distance threshold for RANSAC
    Returns:
        outlier_cloud (open3d.geometry.PointCloud): point cloud without the largest planar component
    '''

    plane_model, inliers = pc.segment_plane(distance_threshold=distance_threshold,
                                            ransac_n=3,
                                            num_iterations=1000)
    [a, b, c, d] = plane_model
    outlier_cloud = pc.select_by_index(inliers, invert=True)
    return outlier_cloud

def transform_pointcloud(msg, target_frame):
    '''
    Transform the point cloud to the target frame
    Args:
        msg (sensor_msgs.msg.PointCloud2): point cloud message
        target_frame (string): target frame
    Returns:
        o3d_pc (open3d.geometry.PointCloud): transformed point cloud
    '''
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    # Get the transform from the camera frame to the world frame
    try: 
        transform = tf_buffer.lookup_transform(target_frame,
                                            msg.header.frame_id,
                                            rospy.Time(0),
                                            rospy.Duration(2.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("Error while transforming the point cloud")
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