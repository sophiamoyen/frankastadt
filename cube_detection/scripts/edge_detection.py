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

import warnings
warnings.filterwarnings("ignore")

SIMULATION = False
STATE = "INIT" #INIT or CHECK

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
    BLACK_TABEL_THRESHOLD = 150
    # threshold to decide weather detected cube is same as before
    MATCH_DISTANCE_THRESHOLD = 0.01
    # for deciding if contour is cube - outdated probably
    MIN_EDGES = 4
    MAX_EDGES = 10
    MIN_AREA = 3800
    MAX_AREA = 5500

class Cube:
    def __init__(self, id, x, y, z, rotation):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.rotation = rotation


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

                if (STATE == "INIT"):
                        self.cubes = detected_cubes
                        # publish cube
                        self.publish_cubes()
                if (STATE == "CHECK"):
                        self.match_detected_with_previous_cubes(detected_cubes)
                        self.publish_cubes()
                if (STATE == "BUILDING"):
                    pass
                
                
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

            if convexity and area > 1500:
            #if True:
            #if (num_edges >= MIN_EDGES) and (num_edges <= MAX_EDGES) and (area >= MIN_AREA) and (area <= MAX_AREA):
            #if len(edges >=4) and len(edges <=7) and abs(cv2.contourArea(contour)) > 50 and abs(cv2.contourArea(contour)) < 2500:
                #cv2.drawContours(self.debug_image, [edges], -1, (255,0,0),2)
                
                if M["m00"] != 0:
                    
                    # Calculate centroid (position)
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    center = np.array([cx, cy])
                    cv2.circle(self.debug_image, (cx, cy), 5, (255, 0, 0), -1)
                    #cube_text = f"Cube {count} - " + text
                    #cube_text = f"Cube {count} - " + text
                    #cv2.putText(self.debug_image, cube_text, (cx, cy -10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

                    depth = self.depth_image[cy, cx]
                    #print("depth ", depth)

                    camera_frame = self.pixel_to_camera_frame(cx, cy, depth)
                    
                    # transform points to world coordinate system
                    transformed_point = self.transform_point(camera_frame)
                    #print(cube_count, ": ", transformed_point)
                    corrected_edges = self.adjust_edge_points(center, edges)
                    transformed_edges = []
                    for edge in corrected_edges:
                        # Assuming `edge` is a point in the contour returned by cv2.approxPolyDP
                        x, y = edge.ravel()  # Convert the point to x, y coordinates
                        #cv2.circle(self.debug_image, (x, y), radius=2, color=(0, 255, 0), thickness=-1)
                        # Assuming you have a method to get depth for each edge point
                        depth = self.depth_image[y, x]

                        # Convert the pixel to camera frame using depth (if your function requires it)
                        camera_frame_point = self.pixel_to_camera_frame(x, y, depth)

                        # Transform the point from the camera frame to the world frame
                        transformed_point = self.transform_point(camera_frame_point)
                        coord_text = f"({transformed_point.point.x:.2f}, {transformed_point.point.y:.2f}, {transformed_point.point.z:.2f}"
                        #cv2.putText(self.debug_image, coord_text, (x + 5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3,(255, 0, 255), 1)
                        # Now `transformed_point` is the point in world coordinates
                        # You might want to store or process these points
                        transformed_edges.append(((x, y), transformed_point))

                    transformed_edges.sort(key=lambda pt: pt[1].point.z, reverse=True)
                    top_edges_pixel_coords = [pt[0] for pt in transformed_edges[:4]]
                    top_contour = np.array(top_edges_pixel_coords).reshape((-1, 1, 2)).astype(np.int32)
                    cv2.drawContours(self.debug_image, [top_contour], -1, (0, 255, 0), 2)


                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)

                    #self.check_cube(cx, cy, box)

                    width = int(rect[1][0])
                    height = int(rect[1][1])
                    angle = int(rect[2])
                    ratio = height/width

                    if width < height:
                       angle = 90 - angle
                    else:
                       angle = -angle

                    cv2.drawContours(self.debug_image,[box],0,(255,0,255),2)

                    cube_text = f"Cube {cube_count} : (" + str(round(width, 2)) + ", " + str(round(height, 2)) + ") " + str(round(height/width, 3)) + " A: " + str(round(area,2))

                    if (area > 6000):
                        cube_text = f"2 Cubes detected {cube_count} : (" + str(round(width, 2)) + ", " + str(round(height, 2)) + ") " + str(round(height/width, 3)) + " A: " + str(round(area,2))

                        # todo: split the cubes
                        # maybe mask everything to validate cubes
                        angle_rad = np.radians(angle)
                        # Calculate direction vector components based on the angle
                        direction_vector = np.array([np.cos(-angle_rad), np.sin(-angle_rad)])

                        move_distance = height / 4
                        if (width > height):
                            move_distance = width / 4
                        
                        
                        # Calculate new centroid positions
                        new_centroid_left = np.array([cx, cy]) - direction_vector * move_distance
                        new_centroid_right = np.array([cx, cy]) + direction_vector * move_distance
                        
                        # For visualization, draw the new centroids on the debug image
                        cv2.circle(self.debug_image, tuple(new_centroid_left.astype(int)), 5, (0, 0, 255), -1)
                        cv2.circle(self.debug_image, tuple(new_centroid_right.astype(int)), 5, (255, 255, 0), -1)

                        

                    #cube_text = f"Cube {cube_count} : (" + str(round(transformed_point.point.x , 2)) + ", " + str(round(transformed_point.point.y, 2)) + ")  orientation: " + str(angle)
                    cv2.putText(self.debug_image, cube_text, (cx-150, cy -10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

                    

                detected_cubes.append(Cube(cube_count, transformed_point.point.x, transformed_point.point.y, transformed_point.point.z, angle))
                cube_count += 1
            else:
                if M["m00"] != 0 and area > 1500:
                    # Calculate centroid (position)
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    
                    width = int(rect[1][0])
                    height = int(rect[1][1])
                    angle = int(rect[2])

                    mask = np.zeros(self.cv_image.shape[:2], dtype=np.uint8)
                    cv2.drawContours(mask, [contour], -1, color=255, thickness=cv2.FILLED)
                    #self.show_image(mask, "Mask", "gray")

                    masked_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)
                    masked_depth = cv2.bitwise_and(self.depth_image, self.depth_image, mask=mask)
                    #self.show_image(masked_image, "Masked Image")

                    x, y, w, h = cv2.boundingRect(edges)
                    non_convex_area = masked_image[y-5:y+h+5, x-5:x+w+5]
                    non_convex_depth = masked_depth[y-5:y+h+5, x-5:x+w+5]

                    self.check_for_cubes(masked_image, masked_depth)
#
            #        if width < height:
            #           angle = 90 - angle
            #        else:
            #           angle = -angle
#
                    cv2.drawContours(self.debug_image,[box],0,(255,255,0),2)
                    cv2.circle(self.debug_image, (cx, cy), 5, (0, 255, 0), -1)
                    cv2.putText(self.debug_image, text, (cx- 250, cy -10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    
        print("detected Cubes: ", len(detected_cubes))
        return detected_cubes
    
    def adjust_edge_points(self, center, edges, correction_distance=11):
        adjusted_edges = []
    
        for edge in edges:
            edge_point = edge[0]  # Get the point from the edge contour format
            vector_to_center = center - edge_point
            # Normalize the vector to have a length of 1
            norm_vector = vector_to_center / np.linalg.norm(vector_to_center)
            # Move the edge point towards the center by the correction distance
            adjusted_point = edge_point + norm_vector * correction_distance
            adjusted_edges.append([adjusted_point.astype(np.int32)])
    
        return np.array(adjusted_edges)
    

    def match_detected_with_previous_cubes(self, detected_cubes):
        cubes_to_publish = []
        matched_cubes_ids = []
        unmatched_cubes = []

        prev_number_of_cubes = len(self.cubes)

        for cube_to_check in detected_cubes:
            matched = False
            for prev_cube in self.cubes:
                distance = self.calculate_distance(cube_to_check, prev_cube)
                #print(prev_cube.id, " dist: ", distance)
                if distance < MATCH_DISTANCE_THRESHOLD:
                    matched = True
                    matched_cube = self.refine_cube_pos(prev_cube, cube_to_check)
                    cubes_to_publish.append(matched_cube)
                    matched_cubes_ids.append(matched_cube.id)
                    break
            if not matched:
                unmatched_cubes.append(cube_to_check)
                # TODO Continue with matching cubes
                # check ids if same length as all cubes.
                # missing id -> unmatched cubeauch 

        missing_ids = self.find_missing_ids(matched_cubes_ids, prev_number_of_cubes)
        print("Cubes on different position: ", missing_ids)

        id_count = 0
        for unmatched_cube in unmatched_cubes:
            if (id_count < len(missing_ids)):
                cubes_to_publish.append(Cube(missing_ids[id_count], unmatched_cube.x, unmatched_cube.y, unmatched_cube.z, unmatched_cube.rotation))
                id_count += 1
            else:
                cubes_to_publish.append(Cube(prev_number_of_cubes, unmatched_cube.x, unmatched_cube.y, unmatched_cube.z, unmatched_cube.rotation))
                prev_number_of_cubes += 1

        self.cubes.clear()
        self.cubes = cubes_to_publish

    def calculate_distance(self, cube1, cube2):
        # Euclidean distance between two cubes
        return ((cube1.x - cube2.x) ** 2 + (cube1.y - cube2.y) ** 2 + (cube1.z - cube2.z) ** 2) ** 0.5

    def refine_cube_pos(self, prev_cube, cube2):
        return Cube(prev_cube.id, (prev_cube.x + cube2.x) / 2, (prev_cube.y + cube2.y) / 2, (prev_cube.z + cube2.z) / 2, ((prev_cube.rotation + cube2.rotation) / 2))

    def find_missing_ids(self, ids_vector, prev_length):
        # Expected set of IDs from 1 to 6
        expected_ids = set(range(0, prev_length))

        # Convert the input vector to a set to remove duplicates and allow for efficient searching
        actual_ids = set(ids_vector)

        # Find the difference between the expected IDs and the actual IDs to find the missing ones
        missing_ids = expected_ids - actual_ids

        # Return the missing IDs as a sorted list (optional)
        return sorted(missing_ids)


    def check_for_cubes(self, img, depth):
        # Convert to float and scale down
        img_float = img.astype(np.float32) / 255.0
        # Apply gamma correction
        gamma_corrected = np.power(img_float, 0.5)  # Gamma < 1 reduces overexposure
        # Scale back to uint8
        corrected_img = np.clip(gamma_corrected * 255, 0, 255).astype(np.uint8)
        # Convert to grayscale
        gray = cv2.cvtColor(corrected_img, cv2.COLOR_BGR2GRAY)
        #self.show_image(gray, "Gray", 'gray')
        #self.show_image(depth, "depth", "gray")
        
        #closest_point = np.unravel_index(np.argmin(depth), depth)
        #print("Top Area Coordinates:", closest_point)

        min_val, _, min_loc, _ = cv2.minMaxLoc(depth)
        depth_tolerance = 0.008
        top_surface_threshold = min_val + depth_tolerance
        _, top_surface_mask = cv2.threshold(depth, top_surface_threshold, 255, cv2.THRESH_BINARY_INV)
        #self.show_image(top_surface_mask, "Top surface Mask", "gray")
        # does not work for edge detection?
        # find 2 perpendicular lines. middle is middle of cube

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (3,3), 0)
        #self.show_image(blurred, "Blur", 'gray')

        #thresholded = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 299, 15)
        adaptive_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, 
                                        cv2.THRESH_BINARY, 11, 2)
        #_, thresholded = cv2.threshold(gray, 20 , 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        #self.show_image(thresholded, "thresholded", 'gray')

        # canny for edge detection
        detected_edges = cv2.Canny(adaptive_thresh, 0, 150)
        #self.show_image(detected_edges, "Detected Edges", 'gray')

        edges_in_top_surface = cv2.bitwise_and(detected_edges, detected_edges, mask=top_surface_mask.astype(np.uint8))
        #self.show_image(edges_in_top_surface, "Top Surface Edges", 'gray')

        contour_image = img.copy()

        contours, hierarchy = cv2.findContours(detected_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        hierarchy = hierarchy[0]  # Get the actual hierarchy array
        for i, contour_info in enumerate(hierarchy):
            next_, previous, first_child, parent = contour_info
            if parent == -1:  # If the contour has no parent, it's a top-level contour
                # Process top-level contour, which might be an individual cube or a group
                contour = contours[i]
                # Example: Draw top-level contours
                cv2.drawContours(contour_image, [contour], -1, (255, 0, 0), 3)
        
        for contour in contours:
            epsilon = 0.01 * cv2.arcLength(contour, True)
            edges = cv2.approxPolyDP(contour, epsilon, True)
            area = abs(cv2.contourArea(contour))
            num_edges = len(edges)
            convexity = cv2.isContourConvex(edges)
            text = f"Num Edges: {num_edges}, Area: {area}, Convex: {convexity}"
            print(text)
            M = cv2.moments(edges)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0
            #if area > 1000:
                #cv2.putText(contour_image, text, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 
                        #0.5, (0, 255, 0), 2)
                #cv2.drawContours(contour_image, contour, -1, (0, 255, 255), 2)  # Draw contours with green color and thickness 2

        # Convert OpenCV image to ROS message
        contour_image_msg = self.bridge.cv2_to_imgmsg(contour_image)
            # Publish
        self.contour_image_publisher.publish(contour_image_msg)

        # Show the image with contours
        #self.show_image(contour_image, "Contours")
        

        # thresholding to isolate non-black objects (table=black)
        #_, thresholded = cv2.threshold(blurred, BLACK_TABEL_THRESHOLD, 255, cv2.THRESH_BINARY)
        #self.show_image(thresholded, "thresholded", 'gray')

        # other theshold trials
        #thresholded = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 299, 15)
        #_, thresholded = cv2.threshold(gray, 0 , 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # makes the outlines thinner
        #kernel = np.ones((5, 5), np.uint8)
        #dilate = cv2.dilate(thresholded, kernel, iterations=1)
        #self.show_image(dilate, "dilate", 'gray')



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
                cube_publisher = rospy.Publisher("cube_{}_odom".format(cube.id), Odometry, queue_size=10)
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