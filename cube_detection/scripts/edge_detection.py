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
BLUR_SIZE = (23, 23)
BLACK_TABEL_THRESHOLD = 120 



class CubeDetector:
    def __init__(self):
        self.cv_image = None
        self.camera_K = None
        self.camera_frame_id = None

        self.depth = None

        self.tf_listener = TransformListener()

        self.bridge = CvBridge()

        self.debug_image = None
        self.debug_image_publisher = rospy.Publisher('/cube_detection/debug_image', Image, queue_size=10)

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
         if self.cv_image is not None and self.depth is not None:
            try:
                print("Running Cube detection")
                self.debug_image = self.cv_image.copy()
                thresholded = self.preprocess_image()

                detected_edges = cv2.Canny(thresholded, 50, 150)
                #self.show_image(detected_edges, "Detected Edges", 'gray')
                contours, _ = cv2.findContours(detected_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                cv2.drawContours(self.debug_image, contours, -1, (255,0,0), 3)
                #self.show_image(contour_image, "Contours")

                detected_cubes = self.find_cubes(contours)

                self.publish_debug_image(self.debug_image)

            except Exception as e:
                print(e)
                

    def preprocess_image(self):
        # Convert to grayscale
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        #self.show_image(gray, "Gray", 'gray')

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, BLUR_SIZE, 0)
        #self.show_image(blurred, "Blur", 'gray')

        # thresholding to isolate non-black objects (table=black)
        _, thresholded = cv2.threshold(blurred, BLACK_TABEL_THRESHOLD, 255, cv2.THRESH_BINARY)
        #self.show_image(thresholded, "thresholded", 'gray')

        # other theshold trials
        #thresholded = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 13, 2)
        #_, thresh = cv2.threshold(gray, 0 , 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        # makes the outlines thinner
        #kernel = np.ones((5, 5), np.uint8)
        #dilate = cv2.dilate(thresholded, kernel, iterations=1)
        #self.show_image(dilate, "dilate", 'gray')

        return thresholded       

    def find_cubes(self, contours):
        detected_cubes = []

        for count, contour in enumerate(contours):
            epsilon = 0.01 * cv2.arcLength(contour, True)
            edges = cv2.approxPolyDP(contour, epsilon, True)

            if len(edges >=4) and len(edges <=8) and (abs(cv2.contourArea(contour)) > 30):
            #if len(edges >=4) and len(edges <=7) and abs(cv2.contourArea(contour)) > 50 and abs(cv2.contourArea(contour)) < 2500:
                cv2.drawContours(self.debug_image, [edges], -1, (0,255,0),2)

                M = cv2.moments(edges)
                if M["m00"] != 0:
                    # Calculate centroid (position)
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(self.debug_image, (cx, cy), 5, (255, 0, 0), -1)
                    cv2.putText(self.debug_image, f"Contour {count}", (cx, cy -10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    #plt.imshow(cv2.cvtColor(debug_image, cv2.COLOR_BGR2RGB))
                    #plt.show()

                    depth = self.depth[cy, cx]
                    print("depth ", depth)
                    camera_frame = self.pixel_to_camera_frame(cx, cy, depth)
                    #print(camera_frame)

                    # transform points to world coordinate system
                    transformed_point = self.transform_point(camera_frame)
                    print(count, ": ", transformed_point)

                    # publish cubes
                    cube_odom = Odometry()
                    cube_odom.header.frame_id = "world"
                    cube_odom.child_frame_id = "world"
                    cube_odom.pose.pose.position.x = transformed_point[0]
                    cube_odom.pose.pose.position.y = transformed_point[1]
                    cube_odom.pose.pose.position.z = 0.0225
                    cube_odom.pose.pose.orientation.x = 0
                    cube_odom.pose.pose.orientation.y = 0
                    cube_odom.pose.pose.orientation.z = 0
                    cube_odom.pose.pose.orientation.w = 0
                    

                    cube_publisher = rospy.Publisher("cube/detection/cube_{}_odom".format(count), Odometry, queue_size=10)
                    cube_publisher.publish(cube_odom)

                detected_cubes.append(contour)
 
        print("detected Cubes: ", len(detected_cubes))
        return detected_cubes
        

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
            self.depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding=depth_msg.encoding)

        except Exception as e:
            print(e)

def main():
    rospy.init_node('cube_detector', anonymous=True)
    cube_detector = CubeDetector()
    rospy.spin()

if __name__ == '__main__':
    main()