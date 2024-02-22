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
    MIN_EDGES = 4
    MAX_EDGES = 10
    MIN_AREA = 50
    MAX_AREA = 3000
else:
    BLUR_SIZE = (27, 27)
    # to filter out the table depends on light conditions
    BLACK_TABEL_THRESHOLD = 150
    # for deciding if contour is cube
    MIN_EDGES = 4
    MAX_EDGES = 10
    MIN_AREA = 3800
    MAX_AREA = 5500

class CubeDetector:
    def __init__(self):
        self.cv_image = None
        self.camera_K = None
        self.camera_frame_id = None

        self.depth_image = None

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
         if self.cv_image is not None and self.depth_image is not None:
            try:
                #max_depth, min_depth = self.calculate_average_depth_from_corners()
                #_, depth_threshold = cv2.threshold(self.depth_image, min_depth, 255, cv2.THRESH_BINARY_INV)

                #masked_rgb = cv2.bitwise_and(self.cv_image, self.cv_image, mask= depth_threshold.astype(np.uint8))

                #self.show_image(self.depth_image, "Depth", "gray")

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
                detected_cubes = self.find_cubes(contours)

                #publish debug image
                self.publish_debug_image(self.debug_image)

            except Exception as e:
                print(e)
                

    def preprocess_image(self):
        # Convert to grayscale
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        #self.show_image(gray, "Gray", 'gray')

        #hist, bins = np.histogram(gray, bins=256, range=[0,256])
        #plt.figure(figsize=(10,4))
        #plt.plot(hist, color="gray")
        #plt.fill_between(range(256), hist, color="gray", alpha=0.3)
        #plt.title("Grayscale histo")
        #plt.xlabel("Pixel Intensity")
        #plt.ylabel("Frequency")
        #plt.xlim([0, 255])
        #plt.show()

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, BLUR_SIZE, 0)
        #self.show_image(blurred, "Blur", 'gray')

        # thresholding to isolate non-black objects (table=black)
        _, thresholded = cv2.threshold(blurred, BLACK_TABEL_THRESHOLD, 255, cv2.THRESH_BINARY)
        #self.show_image(thresholded, "thresholded", 'gray')

        # other theshold trials
        #thresholded = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 299, 15)
        #_, thresholded = cv2.threshold(gray, 0 , 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # makes the outlines thinner
        #kernel = np.ones((5, 5), np.uint8)
        #dilate = cv2.dilate(thresholded, kernel, iterations=1)
        #self.show_image(dilate, "dilate", 'gray')

        return thresholded       

    def find_cubes(self, contours):
        detected_cubes = []

        cube_count = 0

        for count, contour in enumerate(contours):
            epsilon = 0.01 * cv2.arcLength(contour, True)
            edges = cv2.approxPolyDP(contour, epsilon, True)
            area = abs(cv2.contourArea(contour))
            num_edges = len(edges)
            convexity = cv2.isContourConvex(edges)
            text = f"Num Edges: {num_edges}, Area: {area}, Convex: {convexity}"
            M = cv2.moments(edges)

            #if convexity:
            if True:
            #if (num_edges >= MIN_EDGES) and (num_edges <= MAX_EDGES) and (area >= MIN_AREA) and (area <= MAX_AREA):
            #if len(edges >=4) and len(edges <=7) and abs(cv2.contourArea(contour)) > 50 and abs(cv2.contourArea(contour)) < 2500:
                cv2.drawContours(self.debug_image, [edges], -1, (255,0,0),2)

                if M["m00"] != 0:
                    
                    # Calculate centroid (position)
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(self.debug_image, (cx, cy), 5, (255, 0, 0), -1)
                    #cube_text = f"Cube {count} - " + text
                    #cube_text = f"Cube {count} - " + text
                    #cv2.putText(self.debug_image, cube_text, (cx, cy -10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

                    depth = self.depth_image[cy, cx]
                    print("depth ", depth)

                    camera_frame = self.pixel_to_camera_frame(cx, cy, depth)
                    
                    # transform points to world coordinate system
                    transformed_point = self.transform_point(camera_frame)
                    print(cube_count, ": ", transformed_point)

                    
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)

                    self.check_cube(cx, cy, box)

                    width = int(rect[1][0])
                    height = int(rect[1][1])
                    angle = int(rect[2])
                    ratio = height/width

                    if width < height:
                       angle = 90 - angle
                    else:
                       angle = -angle

                    cv2.drawContours(self.debug_image,[box],0,(255,0,255),2)

                    cube_text = f"Cube {cube_count} : (" + str(round(width, 2)) + ", " + str(round(height, 2)) + ") " + str(height/width)

                    if (0.8 > ratio or ratio > 1.2):
                        cube_text = f"2 Cubes detected {cube_count} : (" + str(round(width, 2)) + ", " + str(round(height, 2)) + ") " + str(height/width)

                        # todo: split the cubes
                        # maybe mask everything to validate cubes
                        if (width > height):
                            pass
                        else:
                            pass

                    #cube_text = f"Cube {cube_count} : (" + str(round(transformed_point.point.x , 2)) + ", " + str(round(transformed_point.point.y, 2)) + ")  orientation: " + str(angle)
                    cv2.putText(self.debug_image, cube_text, (cx-150, cy -10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
                    

                    # publish cube
                    self.publish_cube(cube_count, transformed_point, angle)
                    cube_count += 1

                detected_cubes.append(contour)
            #else:
            #    if M["m00"] != 0:
            #        # Calculate centroid (position)
            #        cx = int(M["m10"] / M["m00"])
            #        cy = int(M["m01"] / M["m00"])
            #        rect = cv2.minAreaRect(contour)
            #        box = cv2.boxPoints(rect)
            #        box = np.int0(box)
            #        
            #        width = int(rect[1][0])
            #        height = int(rect[1][1])
            #        angle = int(rect[2])
#
            #        if width < height:
            #           angle = 90 - angle
            #        else:
            #           angle = -angle
#
            #        cv2.drawContours(self.debug_image,[box],0,(255,255,0),2)
            #        cv2.circle(self.debug_image, (cx, cy), 5, (0, 255, 0), -1)
            #        cv2.putText(self.debug_image, text, (cx- 250, cy -10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    
        print("detected Cubes: ", len(detected_cubes))
        return detected_cubes

    def check_cube(self, cx, cy, box):
        pass

    def calculate_average_depth_from_corners(self, corner_size=10):
        h, w = self.depth_image.shape

        corners = [
            self.depth_image[:corner_size, :corner_size], #top left
            self.depth_image[:corner_size, -corner_size:], #top right
            self.depth_image[-corner_size:, :corner_size], #botttom left
            self.depth_image[-corner_size:, -corner_size:], #top right
        ]

        corner_depths = np.hstack([corner.flatten() for corner in corners])
        # todo: handle NaNs or empty pixels

        table_max_depth = np.max(corner_depths)
        table_min_depth = np.min(corner_depths)

        return table_max_depth, table_min_depth
        

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

    def publish_cube(self, id, pos, angle):
        # publish cubes
        cube_odom = Odometry()
        cube_odom.header.frame_id = "world"
        cube_odom.child_frame_id = "cube_{}".format(id)
        cube_odom.pose.pose.position.x = pos.point.x
        cube_odom.pose.pose.position.y = pos.point.y
        cube_odom.pose.pose.position.z = 0.0225
        cube_odom.pose.pose.orientation.x = 0
        cube_odom.pose.pose.orientation.y = 0
        cube_odom.pose.pose.orientation.z = angle
        cube_odom.pose.pose.orientation.w = 0
        
        cube_publisher = rospy.Publisher("cube_{}_odom".format(id), Odometry, queue_size=10)
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