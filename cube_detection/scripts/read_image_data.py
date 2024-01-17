#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf import TransformListener
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
import time


DEBUG = True

#How can i find the top 4 corners?

class Cube:
    def __init__(self, cube_id, edges, position, points_camera_frame):
        self.cube_id = cube_id
        self.edges = edges
        self.position = position
        self.points_camera_frame = points_camera_frame
        self.color = list(np.random.choice(range(256),size=3))

    def update_edges(self, edges):
        self.edges = edges

    def update_position(self, new_position):
        self.position = new_position

    def distance_to(self, other_position):
        return np.linalg.norm(np.array(self.position) - np.array(other_position))


class CubeDetector:
    def __init__(self):
        self.cv_image = None
        self.camera_K = None

        self.depth = None
        self.depth_K = None

        self.target_frame = "world"
        self.camera_frame_id = None
        self.tf_listener = TransformListener()

        self.cubes = []

        self.bridge = CvBridge()
        
        # camera subscriber
        self.image_subscriber = rospy.Subscriber('/zed2/left/image_rect_color', Image, self.imageCallback)
        self.camera_info_subscriber = rospy.Subscriber("/zed2/left/camera_info", CameraInfo, self.cameraInfoCallback) 

        #depth subscriber
        self.depth_subscriber = rospy.Subscriber('/zed2/depth/depth_registered', Image, self.depthCallback)
        self.depth_info_subscriber = rospy.Subscriber('/zed2/depth/camera_info', CameraInfo, self.depthInfoCallback)
        
        # visualization publisher
        self.marker_array_publisher = rospy.Publisher('cube_markers', MarkerArray, queue_size=10)
    


    def pixel_to_camera_frame(self, positions, depths):
        # Camera matrix
        points_camera = []
        
        for position, depth in zip(positions, depths):
        # homogeneous pixel coordinates
            uv_h = np.array([position[0, 0], position[0, 1], 1.0])

        # inverse of camera matrix
            K_inv = np.linalg.inv(self.camera_K)

        # 3D coordinates of point in camera frame
            points_camera.append(np.dot(K_inv, uv_h) * depth)

        return points_camera

    

    def transform_points(self, points, source_frame, target_frame):
        try:
            i = 0
            transformed_points = []
            for point in points:
                stamped_point = PointStamped()
                stamped_point.header.frame_id = source_frame
                stamped_point.header.stamp = rospy.Time(0)
                stamped_point.point.x = point[0]
                stamped_point.point.y = point[1]
                stamped_point.point.z = point[2]
                transformed_points.append(self.tf_listener.transformPoint("/world", stamped_point))
                print(f"transformed z: {transformed_points[i].point.z}")
                i =+ 1
            return transformed_points
        except Exception as e:
            print(e)
            return None

    def preprocessing_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 0 , 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return thresh

    def edge_detection(self, image):
        edges = cv2.Canny(image, 50, 150)
        return edges
    
    def calc_depth(self, depth_values):
        return np.abs(depth_values)
        #return np.multiply(depth_values, self.depth_K[0, 0])


    #docs.opencv.org/4.x/de/dc0/samples_2tapi_2squares_8cpp-example.html#a22
    def find_cubes(self, contours):
        detected_cubes = []

        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            edges = cv2.approxPolyDP(contour, epsilon, True)

            #obere 4 kanten finden?

            #if (len(edges) >= 4) and (len(edges) <= 7) and cv2.isContourConvex(edges) and abs(cv2.contourArea(contour)) > 500:
            if (len(edges) >= 4) and (len(edges) <= 7) and cv2.isContourConvex(edges):
                depth_values = [self.depth[y, x] for [[x, y]] in edges]
                depths = self.calc_depth(depth_values)
                print(f"depth: {depths}")
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
    
    def match_or_create_cube(self, edges, middle, points_camera_frame, threshold= 0.1):
        new_cube = True
        for cube in self.cubes:
            print(f"Cube {cube.cube_id} distance: {cube.distance_to(middle)}")
            if cube.distance_to(middle) < threshold:
                cube.update_edges(edges)
                cube.update_position(middle)
                new_cube = False
                
        if (new_cube == True):
            # neuer Würfel
            new_id = len(self.cubes) + 1
            new_cube = Cube(new_id, edges, middle, points_camera_frame)
            self.cubes.append(new_cube)

    def middle_of_cube(self, positions):
        # Berechnet den Mittelpunkt eines Würfels anhand seiner Eckpunkte
        if len(positions) == 0:
            return None

        sum_x = 0.0
        sum_y = 0.0
        sum_z = 0.0
        for point in positions:
            sum_x += point.point.x
            sum_y += point.point.y
            sum_z += point.point.z

        num_points = len(positions)
        return (sum_x / num_points, sum_y / num_points, sum_z / num_points)

    def process_images(self):
        if self.cv_image is not None and self.depth is not None:
            try:
                preprocessed_image = self.preprocessing_image(self.cv_image)

                edges = self.edge_detection(preprocessed_image)

                contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                detected_cubes = self.find_cubes(contours)

                #if (DEBUG):
                    #plt.imshow(edges, cmap='gray')
                    #plt.title('Threshold Image')
                    #plt.show()
                    #plt.imshow(self.cv_image)
                    #plt.title('Detected Cubes')

                
                for index, detected_cube in enumerate(detected_cubes):
                
                    points_camera_frame = self.pixel_to_camera_frame(detected_cube["edges"], detected_cube["depths"])
                    
                    transformed_positions = self.transform_points(points_camera_frame, self.camera_frame_id, self.target_frame)
                    
                    position = self.middle_of_cube(transformed_positions)

                    self.match_or_create_cube(transformed_positions, position, points_camera_frame)
                    

                    # print(f"Cube {index} in world frame: {transformed_positions}")
                        #print(f"cube {index}: {detected_cube['position']}")
                    cv2.drawContours(self.cv_image, [detected_cube['edges']], -1, (0, 255, 0), 2)
                    M = cv2.moments(detected_cube['edges'])
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        text = f"Cube {index + 1}"
                        cv2.putText(self.cv_image, text, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    #transformed_positions = self.transform_points(Points_camera_frame, '/left_camera_link_optical', '/world')
                    #self.publish_markers(transformed_positions)

                self.publish_cubes()

                if (DEBUG):
                    plt.imshow(cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB))
                    plt.title('Detected Cubes/Contours')
                    plt.show()
                    #plt.legend()
                    #plt.show()
                    #time.sleep(1000)
                    #plt.imshow(self.cv_image)
                    #plt.title('Detected Cubes with Centroids')

                    #for contour in contours:
                        #plt.scatter(contour[:, 0, 0], contour[:, 0, 1], c='green', marker='x', label=f'Cube contours')

                    

                #cv2.imshow("Bild", self.cv_image)
                #cv2.waitKey(0)

            except Exception as e:
                print(e)

    def depthInfoCallback(self, msg):
        self.depth_K = np.array(msg.K).reshape(3,3)

    def cameraInfoCallback(self, msg):
        try:
            self.camera_K = np.array(msg.K).reshape(3,3)
            self.camera_frame_id = msg.header.frame_id
        except Exception as e:
            print(e)

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


    def publish_cubes(self):
        marker_array = MarkerArray()

        for cube in self.cubes:
            # Point-Marker für den Würfel
            point_marker = Marker()
            point_marker.header.frame_id = self.target_frame
            point_marker.id = cube.cube_id
            point_marker.type = Marker.POINTS
            point_marker.action = Marker.ADD
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = 0.02
            point_marker.scale.y = 0.02
            point_marker.color = ColorRGBA(*[float(c) / 255.0 for c in cube.color], 1.0)

            # Position des Würfels
            p = Point()
            p.x, p.y, p.z = cube.position[0], cube.position[1], cube.position[2]
            point_marker.points.append(p)

            for pos in cube.edges:
                p = Point()
                p.x, p.y, p.z = pos.point.x, pos.point.y, pos.point.z
                point_marker.points.append(p)

            marker_array.markers.append(point_marker)

            # Text-Marker für die ID und Koordinaten des Würfels
            text_marker = Marker()
            text_marker.header.frame_id = self.target_frame
            text_marker.id = cube.cube_id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position = Point(p.x, p.y, p.z + 0.1)  # Leicht über dem Punkt
            text_marker.scale.z = 0.02
            text_marker.color = ColorRGBA(*[float(c) / 255.0 for c in cube.color], 1.0)
            text_marker.text = f"Cube {cube.cube_id}: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
            print(f"Cube {cube.cube_id}: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")

            marker_array.markers.append(text_marker)

        self.marker_array_publisher.publish(marker_array)

    def publish_markers(self, transformed_positions):
        marker = Marker()
        marker.header.frame_id = "world"  # oder Ihr Referenz-Koordinatensystem
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Farbe Rot

        for pos in transformed_positions:
            p = Point()
            p.x, p.y, p.z = pos.point.x, pos.point.y, pos.point.z
            marker.points.append(p)

        self.marker_publisher.publish(marker)



def main():
    rospy.init_node('cube_detector', anonymous=True)
    cube_detector = CubeDetector()
    rospy.spin()

if __name__ == '__main__':
    main()