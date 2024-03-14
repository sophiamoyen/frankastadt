#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

from tf import TransformListener
from cv_bridge import CvBridge

import cv2
import numpy as np
import matplotlib.pyplot as plt

SIMULATION = False

#Hyperparameter
if (SIMULATION):
    BLUR_SIZE = (23, 23)
    BLACK_TABEL_THRESHOLD = 32
    MATCH_DISTANCE_THRESHOLD = 0.01
    MIN_EDGES = 4
    MAX_EDGES = 10
    MIN_AREA = 50
    MAX_AREA = 3000
else:
    # helps to eliminate noise on the table
    BLUR_SIZE = (27, 27)
    # to filter out the table depends on light conditions
    BLACK_TABEL_THRESHOLD = 131
    # threshold to decide weather detected cube is same as before
    MATCH_DISTANCE_THRESHOLD = 0.01
    # for deciding if contour is cube - outdated probably
    MIN_EDGES = 4
    MAX_EDGES = 10
    MIN_AREA = 3800
    MAX_AREA = 5500

class Cube:
    def __init__(self, id, x, y, z, rotation, confidence):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.rotation = rotation
        self.confidence = confidence


class CubeDetector:
    def __init__(self):
        self.cubes = []

        self.cv_image = None
        self.camera_K = None
        self.camera_frame_id = None

        self.depth_image = None

        self.tf_listener = TransformListener()

        self.bridge = CvBridge()

        self.debug_image = None
        self.debug_image_publisher = rospy.Publisher('/cube_detection/debug_image', Image, queue_size=10)
        self.contour_image_publisher = rospy.Publisher('/cube_detection/contour_image', Image, queue_size=10)

        if SIMULATION:
            self.target_frame = "world"
            # camera subscriber
            self.image_subscriber = rospy.Subscriber('/zed2/left/image_rect_color', Image, self.imageCallback)
            self.camera_info_subscriber = rospy.Subscriber("/zed2/left/camera_info", CameraInfo, self.cameraInfoCallback) 
            # depth subscriber
            self.depth_subscriber = rospy.Subscriber('/zed2/depth/depth_registered', Image, self.depthCallback)

        else:
            self.target_frame = "world"
            # camera subscriber
            self.image_subscriber = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.imageCallback)
            self.camera_info_subscriber = rospy.Subscriber("/zed2/zed_node/rgb/camera_info", CameraInfo, self.cameraInfoCallback) 
            # depth subscriber
            self.depth_subscriber = rospy.Subscriber('/zed2/zed_node/depth/depth_registered', Image, self.depthCallback)


    def run_cube_detection(self):
         if self.cv_image is not None and self.depth_image is not None:
            try:
                print("Running Cube detection")
                self.debug_image = self.cv_image.copy()
                
                # theshold image
                thresholded = self.preprocess_image()

                # canny for edge detection
                detected_edges = cv2.Canny(thresholded, 50, 150)
                #self.show_image(detected_edges, "Detected Edges", 'gray')
                
                # extracting contours (edges that belong together)
                contours, _ = cv2.findContours(detected_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # draw contours for debugging
                cv2.drawContours(self.debug_image, contours, -1, (255,0,0), 3)
                #self.show_image(contour_image, "Contours")

                # find the cubes + publish
                self.cubes = self.find_cubes(contours)

                # publish cube
                self.publish_cubes()

                #publish debug image
                self.publish_debug_image(self.debug_image)

            except Exception as e:
                print(e)
                

    def preprocess_image(self):
        # Convert to grayscale
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, BLUR_SIZE, 0)
        #self.show_image(blurred, "Blur", 'gray')

        # thresholding to isolate non-black objects (table=black)
        _, thresholded = cv2.threshold(blurred, BLACK_TABEL_THRESHOLD, 255, cv2.THRESH_BINARY)
        #self.show_image(thresholded, "thresholded", 'gray')

        return thresholded       

    def find_cubes(self, contours):
        detected_cubes = []

        cube_count = 0

        for contour in contours:
            epsilon = 0.01 * cv2.arcLength(contour, True)
            edges = cv2.approxPolyDP(contour, epsilon, True)
            area = abs(cv2.contourArea(contour))
            num_edges = len(edges)
            convexity = cv2.isContourConvex(edges)

            # filter everything that is smaller than 1500 (not a cube for sure)
            if area > 1500:
                M = cv2.moments(edges)

                # Calculate centroid (position)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Depth
                depth = self.depth_image[cy, cx]

                # Transform into world frame
                camera_frame = self.pixel_to_camera_frame(cx, cy, depth)
                transformed_point = self.transform_point(camera_frame)

                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                width = int(rect[1][0])
                height = int(rect[1][1])
                angle = int(rect[2])

                if width < height:
                   angle = 90 - angle
                else:
                    angle = -angle

                if convexity:
                    # draw dot in center
                    cv2.circle(self.debug_image, (cx, cy), 5, (255, 0, 0), -1)
                    # draw box around cube
                    cv2.drawContours(self.debug_image,[box],0,(255,0,255),2)
                    # text to display
                    cube_text = f"Cube {cube_count} : (" + str(round(width, 2)) + ", " + str(round(height, 2)) + ") " + str(round(height/width, 3)) + " A: " + str(round(area,2))
                    
                    # declared as 1 Cube
                    if (area < 6500):
                        #cube_text = f"Cube {cube_count} : (" + str(round(transformed_point.point.x , 2)) + ", " + str(round(transformed_point.point.y, 2)) + ")  orientation: " + str(angle)
                        cv2.putText(self.debug_image, cube_text, (cx-150, cy -10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
                        detected_cubes.append(Cube(cube_count, transformed_point.point.x, transformed_point.point.y, transformed_point.point.z, angle, 1))
                        cube_count += 1

                    # if area > 6500 its declared as 2 Cubes
                    else:
                        new_centroid_left, transformed_point_left, new_centroid_right, transformed_point_right = self.split_cubes(cx, cy, width, height, angle)

                        cube_text_left = f"2 Cubes detected {cube_count} : (" + str(round(transformed_point_left.point.x, 2)) + ", " + str(round(transformed_point_left.point.y, 2)) + ")"
                        detected_cubes.append(Cube(cube_count, transformed_point_left.point.x, transformed_point_left.point.y, transformed_point_left.point.z, angle, 0.5))
                        cube_count += 1
                        cube_text_right = f"2 Cubes detected {cube_count} : (" + str(round(transformed_point_right.point.x, 2)) + ", " + str(round(transformed_point_right.point.y, 2)) + ")"
                        detected_cubes.append(Cube(cube_count, transformed_point_right.point.x, transformed_point_right.point.y, transformed_point_right.point.z, angle, 0.5))
                        cube_count += 1

                        cv2.putText(self.debug_image, cube_text_left, (new_centroid_left[0].astype(int)-150, new_centroid_left[1].astype(int)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
                        cv2.putText(self.debug_image, cube_text_right, (new_centroid_right[0].astype(int)-150, new_centroid_right[1].astype(int)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
            
                # not sure about the cube constalation        
                else:

                    detected_cubes.append(Cube(cube_count, transformed_point.point.x, transformed_point.point.y, transformed_point.point.z, angle, 0))
                    mask = np.zeros(self.cv_image.shape[:2], dtype=np.uint8)
                    cv2.drawContours(mask, [contour], -1, color=255, thickness=cv2.FILLED)
                    cv2.drawContours(self.debug_image,[box],0,(255,255,0),2)
                    cv2.circle(self.debug_image, (cx, cy), 5, (0, 255, 0), -1)
                    text = f"Cube unsure: {cube_count}, Area: {area}, Convex: {convexity}, Angle: {angle}"
                    cv2.putText(self.debug_image, text, (cx- 250, cy -10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                    #new_centroid_left, transformed_point_left, new_centroid_right, transformed_point_right = self.split_cubes(cx, cy, width, height, angle)


        print("detected Cubes: ", len(detected_cubes))
        return detected_cubes    

    def split_cubes(self,cx, cy, width, height, angle):
        # maybe mask everything to validate cubes
        angle_rad = np.radians(angle)
        # Calculate direction vector components based on the angle
        direction_vector = np.array([np.cos(-angle_rad), np.sin(-angle_rad)])
        move_distance = height / 4
        if (width > height):
            move_distance = width / 4

        print(angle)
        
        # Calculate new centroid positions
        new_centroid_left = np.array([cx, cy]) - direction_vector * move_distance
        new_centroid_right = np.array([cx, cy]) + direction_vector * move_distance
        
        # For visualization, draw the new centroids on the debug image
        cv2.circle(self.debug_image, tuple(new_centroid_left.astype(int)), 5, (0, 0, 255), -1)
        cv2.circle(self.debug_image, tuple(new_centroid_right.astype(int)), 5, (255, 255, 0), -1)
        depth_left = self.depth_image[new_centroid_left[1].astype(int), new_centroid_left[0].astype(int)]
        depth_right = self.depth_image[new_centroid_right[1].astype(int), new_centroid_right[0].astype(int)]
        # Convert the pixel to camera frame using depth (if your function requires it)
        camera_frame_left_point = self.pixel_to_camera_frame(new_centroid_left[0].astype(int), new_centroid_left[1].astype(int), depth_left)
        camera_frame_right_point = self.pixel_to_camera_frame(new_centroid_right[0].astype(int), new_centroid_right[1].astype(int), depth_right)
        # Transform the point from the camera frame to the world frame
        transformed_point_left = self.transform_point(camera_frame_left_point)
        transformed_point_right = self.transform_point(camera_frame_right_point)

        return new_centroid_left, transformed_point_left, new_centroid_right, transformed_point_right

    def pixel_to_camera_frame(self, x, y, depth):
        uv_h = np.array([x, y, 1.0])
        K_inv = np.linalg.inv(self.camera_K)
        return np.dot(K_inv, uv_h) * depth

    def transform_point(self, point):
        try:
            stamped_point = PointStamped()
            stamped_point.header.frame_id = self.camera_frame_id
            stamped_point.header.stamp = rospy.Time(0)
            stamped_point.point.x = point[0]
            stamped_point.point.y = point[1]
            stamped_point.point.z = point[2]
            transformed_point = self.tf_listener.transformPoint(self.target_frame, stamped_point)
            return transformed_point
        except Exception as e:
            print(e)
            return None

    def show_image(self, img, title, cmap=None):
        plt.imshow(img, cmap=cmap)
        plt.title(title)
        plt.show()

    def publish_debug_image(self, debug_image):
        try:
            # Convert OpenCV image to ROS message
            debug_image_msg = self.bridge.cv2_to_imgmsg(debug_image)
            # Publish
            self.debug_image_publisher.publish(debug_image_msg)
        except Exception as e:
            print(e)

    def publish_cubes(self):
        if self.cubes:
            for cube in self.cubes:
                # publish cubes
                cube_odom = Odometry()
                cube_odom.header.frame_id = "world"
                cube_odom.child_frame_id = "cube_{}".format(cube.id)
                cube_odom.pose.pose.position.x = cube.x
                cube_odom.pose.pose.position.y = cube.y
                cube_odom.pose.pose.position.z = cube.z
                cube_odom.pose.pose.orientation.x = 0
                cube_odom.pose.pose.orientation.y = 0
                cube_odom.pose.pose.orientation.z = cube.rotation
                cube_odom.pose.pose.orientation.w = 0

                print("Publishing Cube {}: ({}, {}, {}) - {}".format(cube.id, round(cube.x, 3), round(cube.y, 3), round(cube.z, 3), round(cube.rotation, 3)))
                cube_publisher = rospy.Publisher("cube_{}_odom_ed".format(cube.id), Odometry, queue_size=10)
                cube_publisher.publish(cube_odom)

    # callbacks
    def imageCallback(self, image_msg):
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding=image_msg.encoding)
                self.run_cube_detection()

            except Exception as e:
                print(e)

    def cameraInfoCallback(self, msg):
            try:
                self.camera_K = np.array(msg.K).reshape(3,3)
                self.camera_frame_id = msg.header.frame_id
            except Exception as e:
                print(e)

    def depthCallback(self, depth_msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding=depth_msg.encoding)

        except Exception as e:
            print(e)


def main():
    rospy.init_node('cube_detector', anonymous=True)
    cube_detector = CubeDetector()
    rospy.spin()

if __name__ == '__main__':
    main()