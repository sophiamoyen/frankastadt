#!/usr/bin/env python3

import rospy
from pick_and_place_module.move_functions import PlanAndMove
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import *
import geometry_msgs.msg
import matplotlib.pyplot as plt
import math
import matplotlib.patches as patches
import sys

class Tower():
    def __init__(self):
        self.plan_and_move = PlanAndMove()
        
        
    def get_detected_cubes(self, n_detected_cubes):
        cube_positions = []
        for i in range(n_detected_cubes):
            pick_position = [rospy.get_param("cube_{}_x".format(i))-0.01,
                             rospy.get_param("cube_{}_y".format(i))-0.05,
                             0.04]
            cube_positions.append(pick_position)

        return(cube_positions)
        


    def find_free_space(self, cube_positions, cube_size, lim_x, lim_y, safety_distance):
        """
        Finds free space for a group of cubes on the table with reserved space in each cube.

        Args:
            cube_positions: List of tuples representing occupied positions (x, y) in cm.
            cube_size: Size of the cube in cm.
            lim_x: Tuble (x_min,x_max) of table
            lim_y: Tuple (y_min,y_max) of table
            safety_distance: Min distance from center of cube to consider free space

        Returns:
            free_space: List of tuples representing free space positions (x, y) in cm.
            occupied_space: List of tuples representing occupied space positions (x, y) in cm.
        """
        free_positions = []
        impossible_positions = []
        x_min = lim_x[0]
        x_max = lim_x[1]
        y_min = lim_y[0]
        y_max = lim_y[1]

        x_coords = [x * cube_size for x in range(int(x_min // cube_size)+1, int(x_max // cube_size)+1)]
        y_coords = [y * cube_size for y in range(int(y_min // cube_size)+1, int(y_max // cube_size)+1)]
        x_limits = [x_coords[0], x_coords[-1]]
        y_limits = [y_coords[0], y_coords[-1]]

        for y in y_coords:
            for x in x_coords:
                grid_pos = (x,y)
                free = True
                
                for cube_pos in cube_positions:
                    if (math.dist(grid_pos,cube_pos) <= safety_distance) or (x in x_limits) or (y in y_limits):
                        free = False
                        break

                if free:
                    free_positions.append(grid_pos)

                if not free:
                    impossible_positions.append(grid_pos)


        return free_positions, impossible_positions

    def plot_free_space(self, free_positions, impossible_positions, cube_size, cube_positions, lim_x, lim_y, safety_distance, cube_indexes):
        # Plot occupied and free spaces
        fig, ax = plt.subplots(figsize=((8,10)))

        for x, y in free_positions:
            # Plot free space
            plt.plot(x, y, 'o', markersize=4, color='lightgreen')
        for x, y in impossible_positions:
            # Plot impossible positions
            plt.plot(x, y, 'o', markersize=4, color='red')

        colors = "bgrcmykw"
        i = 0
        for x, y in cube_positions:
            # Plot cubes
            square = patches.Rectangle((x-0.0275, y-0.0275), 0.045, 0.045, edgecolor='black', facecolor=colors[cube_indexes[i]], label="cube_{}".format(cube_indexes[i]))
            Drawing_colored_circle = plt.Circle((x, y), safety_distance, fill = False)
            ax.add_artist( Drawing_colored_circle )
            ax.add_patch(square)
            plt.gca().set_aspect('equal', adjustable='box')
            i += 1
            
        plt.plot(.5,0,'*', markersize=20, color="black", label="Origin")
        plt.xlim(*lim_x)
        plt.ylim(*lim_y)
        plt.xlabel("Y (m)")
        plt.ylabel("X (m)")
        plt.title("Occupancy Grid")
        plt.legend(loc='upper right')
        plt.show()

    def plot_tower_place(self, free_positions, impossible_positions, cube_size, cube_positions, 
                         lim_x, lim_y, safety_distance, cube_indexes, cubes_tower_pos):
        # Plot occupied and free spaces
        fig, ax = plt.subplots(figsize=((8,10)))

        for x, y in free_positions:
            # Plot free space
            plt.plot(x, y, 'o', markersize=4, color='lightgreen')
        for x, y in impossible_positions:
            # Plot impossible positions
            plt.plot(x, y, 'o', markersize=4, color='red')

        colors = "bgrcmykw"
        i = 0
        for x, y in cube_positions:
            # Plot cubes
            square = patches.Rectangle((x-0.0275, y-0.0275), 0.045, 0.045, edgecolor='black', facecolor=colors[cube_indexes[i]], label="cube_{}".format(cube_indexes[i]))
            Drawing_colored_circle = plt.Circle((x, y), safety_distance, fill = False)
            ax.add_artist( Drawing_colored_circle )
            ax.add_patch(square)
            plt.gca().set_aspect('equal', adjustable='box')
            i += 1

        i = 0
        for cube_pos in cubes_tower_pos[:2]:
            tower = patches.Rectangle((cube_pos[0]-0.0275, cube_pos[1]-0.0275), 0.045, 0.045, edgecolor='black', facecolor='gold', label="tower_cube_{}".format(i))
            Drawing_colored_circle = plt.Circle((cube_pos[0], cube_pos[1]), safety_distance, fill = False)
            ax.add_artist( Drawing_colored_circle )
            ax.add_patch(tower)
            plt.gca().set_aspect('equal', adjustable='box')
            i += 1
            
        plt.plot(0.5,0,'*', markersize=20, color="black", label="Origin")
        plt.xlim(*lim_x)
        plt.ylim(*lim_y)
        plt.xlabel("Y (m)")
        plt.ylabel("X (m)")
        plt.title("Occupancy Grid")
        plt.legend(loc='upper right')
        plt.show()

    def find_closest_cube_origin(self, cube_positions, desired_center, indexes):
        closest_distance_origin = 1000 # Set distance very high

        i = 0
        for cube_pos in cube_positions:
            distance_origin = math.dist(cube_pos,desired_center)
            if distance_origin < closest_distance_origin:
                # Checks if this cube is closer to the desired center than the previous ones
                closest_distance_origin = distance_origin
                closest_cube = cube_pos
                closest_cube_index = indexes[i]
            i += 1
            
        print("Closest cube to origin: cube_{} with distance ".format(closest_cube_index),closest_distance_origin)
        return closest_cube, closest_cube_index

    def creates_tower3_structure(self, desired_place, orientation="vertical"):
        # Add z coordinate
        #desired_place = [*desired_place,0.04] # Real world
        desired_place = [*desired_place,rospy.get_param("cube_0_z")]

        # Create list of poses for cubes in tower
        cubes_tower_pos = [desired_place]


        if orientation == "vertical":
            cubes_tower_pos.append([desired_place[0]+0.055,desired_place[1],0.04]) # Real world
            #cubes_tower_pos.append([desired_place[0]+0.045,desired_place[1],rospy.get_param("cube_0_z")]) # Simulation
            cubes_tower_pos.append([desired_place[0]+0.0275,desired_place[1],0.1]) # Real world
            #cubes_tower_pos.append([desired_place[0]+0.0275,desired_place[1],rospy.get_param("cube_0_z")+0.1]) # Simulation

        if orientation == "horizontal":
            cubes_tower_pos.append([desired_place[0],desired_place[1]+0.045,0.04])
            cubes_tower_pos.append([desired_place[0],desired_place[1]+0.0275,0.1])

        return(cubes_tower_pos)

    def check_possible_tower_place(self, cubes_tower_pos, impossible_positions, safety_distance):
        base_cubes = cubes_tower_pos[:2] # For tower of 3 cubes, gets the 2 base cubes of the tower

        placement_possible = True
        for cube_pos in base_cubes:
            cube_pos = cube_pos [:2] # Doesn't consider z coordinate for safety check
            for imp_pos in impossible_positions:
                if math.dist(cube_pos,imp_pos) < safety_distance:
                    print("----------- Impossible tower placement --------------")
                    placement_possible = False
                    break
        return placement_possible



    def pickplace(self):
        self.plan_and_move.move_standard_pose()
        
        
        pick_position = [rospy.get_param("cube_0_x")-0.01,
                         rospy.get_param("cube_0_y")-0.05,
                         0.05]
        """
        pick_position = [0.48, -0.32, 0.04]
        """
        print("pick_position",pick_position)
        

        pick_orientation = [0.9239002820650952,  
                            -0.3826324133679813, 
                            -0.000784053224384248,  
                            0.00030050087016984296]

        place_position = [rospy.get_param("cube_1_x")-0.01,
                          rospy.get_param("cube_1_y")-0.05,
                          0.12]

        self.plan_and_move.setPickPose(*pick_position,*pick_orientation)
        self.plan_and_move.setPlacePose(*place_position,*pick_orientation)
        self.plan_and_move.execute_pick()
        self.plan_and_move.execute_place()
        self.plan_and_move.move_standard_pose()

    def collision_scene(self):
        # Creates collision scene for table and around the robot
        
        """
        self.plan_and_move.moveit_control.scene.remove_world_object("left_bar")
        self.plan_and_move.moveit_control.scene.remove_world_object("right_bar")
        self.plan_and_move.moveit_control.scene.remove_world_object("front_bar")
        self.plan_and_move.moveit_control.scene.remove_world_object("table")
        """

        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "panda_link0"
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.x = 0.495
        table_pose.pose.position.y = 0
        table_pose.pose.position.z = -0.3935
        self.plan_and_move.moveit_control.scene.add_box("table", table_pose, size=(0.81, 1.49, 0.787))

        left_bar_pose = geometry_msgs.msg.PoseStamped()
        left_bar_pose.header.frame_id = "panda_link0"
        left_bar_pose.pose.orientation.w = 1.0
        left_bar_pose.pose.position.x = 0.495
        left_bar_pose.pose.position.y = 0.795
        left_bar_pose.pose.position.z = 0.4
        self.plan_and_move.moveit_control.scene.add_box("left_bar", left_bar_pose, size=(0.81, 0.1, 1.6))

        right_bar_pose = geometry_msgs.msg.PoseStamped()
        right_bar_pose.header.frame_id = "panda_link0"
        right_bar_pose.pose.orientation.w = 1.0
        right_bar_pose.pose.position.x = 0.495
        right_bar_pose.pose.position.y = -0.795
        right_bar_pose.pose.position.z = 0.4
        self.plan_and_move.moveit_control.scene.add_box("right_bar", right_bar_pose, size=(0.81, 0.1, 1.6))

        front_bar_pose = geometry_msgs.msg.PoseStamped()
        front_bar_pose.header.frame_id = "panda_link0"
        front_bar_pose.pose.orientation.w = 1.0
        front_bar_pose.pose.position.x = 0.86
        front_bar_pose.pose.position.y = 0
        front_bar_pose.pose.position.z = 0.4
        self.plan_and_move.moveit_control.scene.add_box("front_bar", front_bar_pose, size=(0.1, 1.49, 1.6))
        
    def stack(self, place_position, pick_position):
        self.plan_and_move.move_standard_pose()
        """ 
        --- Picking ---
        Assumes that z axis is always turned upwards! 
        ---------------------------------------------
        """

        pick_orientation = [0.9239002820650952,  
                            -0.3826324133679813, 
                            -0.000784053224384248,  
                            0.00030050087016984296]

        """ 
        --- Execution ---
        """
        self.plan_and_move.setPickPose(*pick_position,*pick_orientation)
        self.plan_and_move.setPlacePose(*place_position,*pick_orientation)
        self.plan_and_move.execute_pick()
        self.plan_and_move.execute_place()

    def task_3_cubes(self):
        cube0_position = [0.5-0.01,-0.05,0.05]
        cube1_position = [0.55-0.01,-0.05,0.05]
        cube2_position = [0.53-0.01,-0.05,0.11]

        list_cubes_pos = []
        
        for i in range(3):
            pick_position = [rospy.get_param("cube_{}_x".format(i))-0.01,
                            rospy.get_param("cube_{}_y".format(i))-0.05,
                            0.04]
            list_cubes_pos.append(pick_position)

        list_cubes = [cube0_position,
                      cube1_position,
                      cube2_position]

        self.plan_and_move.move_standard_pose()

        i = 0
        for cube_pos in list_cubes:
            self.stack(cube_pos,list_cubes_pos[i])
            i += 1

        self.plan_and_move.move_standard_pose()

    def task(self):
        cube0_position = [0.5,0,rospy.get_param("cube_0_z")]
        cube1_position = [0.545,0,rospy.get_param("cube_0_z")]
        cube2_position = [0.455,0,rospy.get_param("cube_0_z")]
        cube3_position = [0.4775,0,rospy.get_param("cube_0_z")+0.05]
        cube4_position = [0.5225,0,rospy.get_param("cube_0_z")+0.05]
        cube5_position = [0.5,0,rospy.get_param("cube_0_z")+0.1]

        list_cubes = [cube0_position,
                    cube1_position,
                    cube2_position,
                    cube3_position,
                    cube4_position,
                    cube5_position]

        self.plan_and_move.move_standard_pose()

        i = 0
        for cube_pos in list_cubes:
            self.stack(i,cube_pos)
            i += 1

        self.plan_and_move.move_standard_pose()


if __name__ == "__main__":
    cla = Tower()
    cla.collision_scene()
    cla.plan_and_move.move_standard_pose()





    #cube_positions = [[0.63, 0.26],[0.42,0.09],[0.29,-0.20],[0.63,-0.22],[0.43,-0.31]]
    #cube_indexes = [0,1,2,3,4]
    cube_size = 0.045  # cm
    lim_y = [-0.50,0.50]
    lim_x = [0.10,0.80]
    safety_distance = (cube_size*math.sqrt(2))+0.01
    desired_center = [0.50,0]
    n_detected_cubes = 3

    
    cube_positions = []
    cube_indexes = []
    for i in range(n_detected_cubes):
        pick_position = [rospy.get_param("cube_{}_x".format(i))-0.015,
                         rospy.get_param("cube_{}_y".format(i))-0.05]
        cube_positions.append(pick_position)
        cube_indexes.append(i)

    free_positions, impossible_positions = cla.find_free_space(cube_positions, 
                                                               cube_size, 
                                                               lim_x, 
                                                               lim_y, 
                                                               safety_distance)

    cla.plot_free_space(free_positions, 
                        impossible_positions, 
                        cube_size, 
                        cube_positions, 
                        lim_x, 
                        lim_y, 
                        safety_distance,
                        cube_indexes)

    closest_cube, closest_cube_index = cla.find_closest_cube_origin(cube_positions, desired_center, cube_indexes)
    print("========== Removed cube_{} from occupied space".format(closest_cube_index))


    cubes_to_move = cube_positions.copy()
    cubes_to_move_indexes = cube_indexes.copy()
    cubes_to_move_indexes.remove(closest_cube_index)
    cubes_to_move.remove(closest_cube)

    free_positions, impossible_positions = cla.find_free_space(cubes_to_move, 
                                                               cube_size, 
                                                               lim_x, 
                                                               lim_y, 
                                                               safety_distance)

    cla.plot_free_space(free_positions, 
                        impossible_positions, 
                        cube_size, 
                        cubes_to_move, 
                        lim_x, 
                        lim_y, 
                        safety_distance,
                        cubes_to_move_indexes)
    
    print("================= Generating tower strucutre with 3 cubes with vertical orientation starting from position of cube_{}:".format(closest_cube_index), closest_cube)
    cubes_tower_pos = cla.creates_tower3_structure(closest_cube, orientation="vertical")
    print("================= Tower strucure generated:",cubes_tower_pos)
    placement_possible = cla.check_possible_tower_place(cubes_tower_pos, impossible_positions, safety_distance)
    print("================= Placement possible:",placement_possible)
    
    if placement_possible == False:
        sys.exit(1)


    cla.plot_tower_place(free_positions, impossible_positions, cube_size, cubes_to_move, 
                         lim_x, lim_y, safety_distance, cubes_to_move_indexes, cubes_tower_pos)

    # Getting Pick  SIMULATION
    pick_positions = [[*closest_cube,rospy.get_param("cube_0_z")]]
    tower_closest_cube, closest_cube_index = cla.find_closest_cube_origin(cubes_to_move, closest_cube, cubes_to_move_indexes)
    #pick_positions.append([*tower_closest_cube,rospy.get_param("cube_0_z")]) # Simulation
    pick_positions.append([*tower_closest_cube,0.04])
    cubes_to_move_indexes.remove(closest_cube_index)
    cubes_to_move.remove(tower_closest_cube)
    tower_closest_cube, closest_cube_index = cla.find_closest_cube_origin(cubes_to_move, closest_cube, cubes_to_move_indexes)
    #pick_positions.append([*tower_closest_cube,rospy.get_param("cube_0_z")]) # Simulation
    pick_positions.append([*tower_closest_cube,0.04])

    print("=========== Place positions:",cubes_tower_pos)
    print("=========== Pick positions:", pick_positions)



    for i in range(1,3):
        cla.stack(cubes_tower_pos[i],pick_positions[i])


    cla.plan_and_move.move_standard_pose()
    

