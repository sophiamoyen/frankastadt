#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
from tf import TransformListener
from geometry_msgs.msg import PointStamped
import tf2_ros
import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
import time


DEBUG = True

#How can i find the top 4 corners?

class CubeDetector:
    def __init__(self):
        self.cv_image = None
        self.depth = None
        self.camera_K = None
        self.depth_K = None

        self.tf_listener = TransformListener()

        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber('/zed2/left/image_rect_color', Image, self.imageCallback)
        self.depth_subscriber = rospy.Subscriber('/zed2/depth/depth_registered', Image, self.depthCallback)
        self.depth_subscriber = rospy.Subscriber('/zed2/depth/camera_info', CameraInfo, self.depthInfoCallback)
        self.camera_info_subscriber = rospy.Subscriber("/zed2/left/image_rect_color/camera_info", CameraInfo, self.cameraInfoCallback) 
        #self.point_cloud_subscriber = rospy.Subscriber("/zed2/point_cloud/cloud_registered", PointCloud2, self.pointCloudCallback)       
        #self.runprocessing()
        #self.ts = message_filters.TimeSynchronizer([self.image_subscriber, self.depth_subscriber], 10)
        #self.ts.registerCallback(self.tsCallback)
    def depthInfoCallback(self, msg):
        self.depth_K = np.array(msg.K).reshape(3,3)


    def pixel_to_camera_frame(self, positions, depths):
        # Camera matrix
        points_camera = []
        K = np.reshape(self.camera_K, (3, 3))
        for position, depth in zip(positions, depths):
        # homogeneous pixel coordinates
            uv_h = np.array([position[0, 0], position[0, 1], 1.0])

        # inverse of camera matrix
            K_inv = np.linalg.inv(K)

        # 3D coordinates of point in camera frame
            points_camera.append(np.dot(K_inv, uv_h) * depth)

        return points_camera

    

    def transform_points(self, points, source_frame, target_frame):
        try:
            transformed_points = []
            for point in points:
                stamped_point = PointStamped()
                stamped_point.header.frame_id = source_frame
                stamped_point.header.stamp = rospy.Time(0)
                stamped_point.point.x = point[0]
                stamped_point.point.y = point[1]
                transformed_points.append(self.tf_listener.transformPoint(target_frame, stamped_point))
            return transformed_points
        except Exception as e:
            print(e)
            return None

    def preprocessing_image(self, image):
        #todo maybe
        return image

    def edge_detection(self, image):
        edges = cv2.Canny(image, 50, 150)
        return edges
    
    def calc_depth(self, depth_values):
        return depth_values
        #return np.multiply(depth_values, self.depth_K[0, 0])


    #docs.opencv.org/4.x/de/dc0/samples_2tapi_2squares_8cpp-example.html#a22
    def find_cubes(self, contours):
        detected_cubes = []

        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            edges = cv2.approxPolyDP(contour, epsilon, True)

            #obere 4 kanten finden?

            #if (len(edges) >= 4) and (len(edges) <= 7) and cv2.isContourConvex(edges) and abs(cv2.contourArea(contour)) > 500:
            if (len(edges) >= 4) and (len(edges) <= 7):
                depth_values = [self.depth[y, x] for [[x, y]] in edges]
                depths = self.calc_depth(depth_values)
                average_depth = sum(depth_values) / len(depth_values)
    
                M = cv2.moments(edges)
                if M["m00"] != 0:
                    # Calculate centroid (position)
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Calculate orientation
                    orientation_rad = 0.5 * math.atan2(2 * M["m11"] - M["m20"] - M["m02"], M["m20"] - M["m02"])
                    orientation_deg = math.degrees(orientation_rad)

                detected_cubes.append({
                    'edges': edges,
                    'depths': depths,

                }) 

        return detected_cubes

    def process_images(self):
        if self.cv_image is not None and self.depth is not None:
            try:
                preprocessed_image = self.preprocessing_image(self.cv_image)

                edges = self.edge_detection(preprocessed_image)

                contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                #if (DEBUG):
                 #   img_copy = self.cv_image.copy()
                  #  cv2.drawContours(img_copy, contours, -1, (0,255,0), 3)
                   # cv2.imshow("contours", img_copy)
                    #cv2.waitKey(0)

                #for debugging
                colors = [(255,0,0), (0,255,0), (0,0,255), (255,255,0), (0,255,255)]

                detected_cubes = self.find_cubes(contours)

                if (DEBUG):
                    plt.imshow(self.cv_image)
                    plt.title('Detected Cubes')

                
                for index, detected_cube in enumerate(detected_cubes):
                    
                    if (DEBUG):
                        print(f"detected cubes: {len(detected_cubes)}")
                        
                        #img_copy = self.cv_image.copy()
                        #img = cv2.circle(img_copy, detected_cube["position"], radius=1, color=(0,0,255), thickness=-1)
                        #cv2.imshow("position", img)
                        #cv2.waitKey(0)

                        plt.scatter(detected_cube['edges'][:, 0, 0], detected_cube['edges'][:, 0, 1], c='blue', marker='x', label=f'Cube {index} contour')
                        #plt.scatter(detected_cube['position'][0], detected_cube['position'][1], c='red', marker='x', label=f'Cube {index} centroid')



                    Points_camera_frame = self.pixel_to_camera_frame(detected_cube["edges"], detected_cube["depths"])
                    
                    transformed_positions = self.transform_points(Points_camera_frame, '/left_camera_link_optical', '/world')
                    print(f"Cube {index} in world frame: {transformed_positions}")
                        #print(f"cube {index}: {detected_cube['position']}")
                        #cv2.drawContours(self.cv_image, detected_cube["contour"], -1, colors[index], 3)
                
                if (DEBUG):
                    plt.legend()
                    plt.show()
                    time.sleep(1000)
                    #plt.imshow(self.cv_image)
                    #plt.title('Detected Cubes with Centroids')

                    #for contour in contours:
                        #plt.scatter(contour[:, 0, 0], contour[:, 0, 1], c='green', marker='x', label=f'Cube contours')

                    

                #cv2.imshow("Bild", self.cv_image)
                #cv2.waitKey(0)

            except Exception as e:
                print(e)


    def cameraInfoCallback(self, msg):
        self.camera_K = msg.K

    def imageCallback(self, image_msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding=image_msg.encoding)
            self.process_images()

        except Exception as e:
            print(e)

    def depthCallback(self, depth_msg):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding=depth_msg.encoding)
            self.process_images()

        except Exception as e:
            print(e)


def main():
    rospy.init_node('cube_detector', anonymous=True)
    cube_detector = CubeDetector()
    rospy.spin()

if __name__ == '__main__':
    main()