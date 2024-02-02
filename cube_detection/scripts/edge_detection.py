#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge

import cv2
import numpy as np
import matplotlib.pyplot as plt

SIMULATION = False

#Hyperparameter
BLUR_SIZE = (7, 7)
BLACK_TABEL_THRESHOLD = 100 


class CubeDetector:
    def __init__(self):
        self.cv_image = None
        self.camera_K = None
        self.camaera_frame_id = None

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

    def run_cube_detection(self):
         if self.cv_image is not None:
            try:
                
                thresholded = self.preprocess_image()
                detected_edges = cv2.Canny(thresholded, 50, 100)
                #self.show_image(detected_edges, "Detected Edges", 'gray')
                contours, _ = cv2.findContours(detected_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                contour_image = self.cv_image.copy()
                cv2.drawContours(contour_image, contours, -1, (0,255,0), 3)
                #self.show_image(contour_image, "Contours")

                detected_cubes = self.find_cubes(contours)


            except Exception as e:
                print(e)
                

    def preprocess_image(self):
        # Convert to grayscale
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        #self.show_image(gray, "Gray", 'gray')
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, BLUR_SIZE, 0)

        # Adaptive thresholding to isolate non-black objects
        _, thresholded = cv2.threshold(blurred, BLACK_TABEL_THRESHOLD, 255, cv2.THRESH_BINARY)

        self.show_image(thresholded, "Thresholded", 'gray')

        self.fill_top_right(thresholded)
        self.show_image(thresholded, "TOP", 'gray')

        return thresholded       
    
    def fill_top_right(self, image):
        """
        Starts from the top right corner, turning all neighboring white pixels to black
        until it hits the first black pixel, signifying the edge of the table.
        """
        rows, cols = image.shape
        for y in range(100):  # Assuming the gripper is within the top 100 rows
            for x in range(cols - 1, -1, -1):  # Start from the right
                if image[y, x] == 255:  # If the pixel is white
                    image[y, x] = 0  # Turn it to black
                else:
                    break 

    def find_cubes(self, contours):
        etected_cubes = []
        debug_image = self.cv_image.copy()

        for contour in contours:
            epsilon = 0.01 * cv2.arcLength(contour, True)
            edges = cv2.approxPolyDP(contour, epsilon, True)

            print(cv2.contourArea(contour))

            if len(edges >=4) and len(edges <=7) and abs(cv2.contourArea(contour)) > 50 and abs(cv2.contourArea(contour)) < 2500:
                cv2.drawContours(debug_image, [edges], -1, (0,255,255),2)

        self.show_image(debug_image, "cubes")

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

def main():
    rospy.init_node('cube_detector', anonymous=True)
    cube_detector = CubeDetector()
    rospy.spin()

if __name__ == '__main__':
    main()