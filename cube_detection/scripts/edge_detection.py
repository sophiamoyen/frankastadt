#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge

import cv2
import numpy as np
import matplotlib.pyplot as plt

SIMULATION = False

#Hyperparameter
BLUR_SIZE = (23, 23)
BLACK_TABEL_THRESHOLD = 150 


class CubeDetector:
    def __init__(self):
        self.cv_image = None
        self.camera_K = None
        self.camera_frame_id = None

        self.depth = None

        self.bridge = CvBridge()

        if SIMULATION:
            self.target_frame = "world"
            # camera subscriber
            self.image_subscriber = rospy.Subscriber('/zed2/left/image_rect_color', Image, self.imageCallback)
            self.camera_info_subscriber = rospy.Subscriber("/zed2/left/camera_info", CameraInfo, self.cameraInfoCallback) 
        else:
            self.target_frame = "map"
            # camera subscriber
            self.image_subscriber = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.imageCallback)
            self.camera_info_subscriber = rospy.Subscriber("/zed2/zed_node/rgb/camera_info", CameraInfo, self.cameraInfoCallback) 
            # depth subscriber
            self.depth_subscriber = rospy.Subscriber('/zed2/zed_node/depth/depth_registered', Image, self.depthCallback)

    def run_cube_detection(self):
         if self.cv_image is not None and self.depth is not None:
            try:
                thresholded = self.preprocess_image()
                detected_edges = cv2.Canny(thresholded, 50, 150)
                #self.show_image(detected_edges, "Detected Edges", 'gray')
                contours, _ = cv2.findContours(detected_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                contour_image = self.cv_image.copy()
                cv2.drawContours(contour_image, contours, -1, (0,255,255), 3)
                self.show_image(contour_image, "Contours")

                detected_cubes = self.find_cubes(contours)


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
        
        self.show_image(thresholded, "thresholded", 'gray')
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
        debug_image = self.cv_image.copy()

        for count, contour in enumerate(contours):
            epsilon = 0.01 * cv2.arcLength(contour, True)
            edges = cv2.approxPolyDP(contour, epsilon, True)

            if len(edges >=4) and len(edges <=7) and (abs(cv2.contourArea(contour)) > 20):
            #if len(edges >=4) and len(edges <=7) and abs(cv2.contourArea(contour)) > 50 and abs(cv2.contourArea(contour)) < 2500:
                cv2.drawContours(debug_image, [edges], -1, (0,255,0),2)

                M = cv2.moments(edges)
                if M["m00"] != 0:
                    # Calculate centroid (position)
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    depth = self.depth[cy, cx]
                    print(depth)

                    cv2.circle(debug_image, (cx, cy), 5, (255, 0, 0), -1)
                    cv2.putText(debug_image, f"Contour {depth}", (cx, cy -10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)


                detected_cubes.append(contour)

        print(len(detected_cubes))
        plt.imshow(cv2.cvtColor(debug_image, cv2.COLOR_BGR2RGB))
        plt.show()

    def show_image(self, img, title, cmap=None):
        plt.imshow(img, cmap=cmap)
        plt.title(title)
        plt.show()

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