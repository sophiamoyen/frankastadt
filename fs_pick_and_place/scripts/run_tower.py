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
from tower import Tower

if __name__ == "__main__":
    cla = Tower()

    cube_size = 0.045  # cm
    lim_y = [-0.50,0.50]
    lim_x = [0.10,0.80]
    safety_distance = (cube_size*math.sqrt(2))
    desired_center = [0.50,0]
    n_detected_cubes = 6

    
    cube_positions = []
    cube_indexes = []
    for i in range(n_detected_cubes):
        pick_position = [rospy.get_param("cube_{}_x".format(i)),
                         rospy.get_param("cube_{}_y".format(i))]
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
    pick_positions.append([*tower_closest_cube,rospy.get_param("cube_0_z")])
    cubes_to_move_indexes.remove(closest_cube_index)
    cubes_to_move.remove(tower_closest_cube)
    tower_closest_cube, closest_cube_index = cla.find_closest_cube_origin(cubes_to_move, closest_cube, cubes_to_move_indexes)
    pick_positions.append([*tower_closest_cube,rospy.get_param("cube_0_z")])

    print("=========== Place positions:",cubes_tower_pos)
    print("=========== Pick positions:", pick_positions)



    for i in range(1,3):
        cla.stack(cubes_tower_pos[i],pick_positions[i])


    cla.plan_and_move.move_standard_pose()