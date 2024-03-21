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

    cla.plan_and_move.move_standard_pose()
    """
    n_detected_cubes = int(rospy.get_param("num_cubes"))
    desired_center = (0.5,0)

    
    cube_poses = []
    cube_yaws = []
    cube_indexes = []
    for i in range(n_detected_cubes):
        pick_pose = [rospy.get_param("cube_{}_x".format(i)),
                     rospy.get_param("cube_{}_y".format(i)),
                     cla.convert_orientation(rospy.get_param("cube_{}_orient_z".format(i)))]
        cube_yaws.append(rospy.get_param("cube_{}_orient_z".format(i)))
        cube_poses.append(pick_pose)
        cube_indexes.append(i)




    free_positions, impossible_positions = cla.find_free_space(cube_poses)

    cla.plot_free_space(free_positions, 
                        impossible_positions, 
                        cube_poses,
                        cube_yaws,
                        cube_indexes)

    closest_cube, closest_cube_index = cla.find_closest_cube(cube_poses, desired_center, cube_indexes)
    print("========== Removed cube_{} from occupied space".format(closest_cube_index))


    cubes_to_move = cube_poses.copy()
    cubes_to_move_indexes = cube_indexes.copy()
    cubes_to_move_indexes.remove(closest_cube_index)
    print("cubes_to_move",cubes_to_move)
    print("cubes_to_move",closest_cube)
    cubes_to_move.remove(closest_cube)

    free_positions, impossible_positions = cla.find_free_space(cubes_to_move)

    cla.plot_free_space(free_positions, 
                        impossible_positions, 
                        cubes_to_move, 
                        cube_yaws,
                        cubes_to_move_indexes)
    
    print("================= Generating tower strucutre with 3 cubes with vertical orientation starting from position of cube_{}:".format(closest_cube_index), closest_cube)
    cubes_tower_pos = cla.creates_tower3_structure(closest_cube, orientation="horizontal")
    print("================= Tower strucure generated:",cubes_tower_pos)
    placement_possible = cla.check_possible_tower_place(cubes_tower_pos, impossible_positions)
    print("================= Placement possible:",placement_possible)
    
    if placement_possible == False:
        sys.exit(1)


    cla.plot_tower_place(free_positions, impossible_positions, cubes_to_move, cube_yaws, cubes_to_move_indexes, cubes_tower_pos)

    # Getting Pick  SIMULATION
    pick_poses = [[closest_cube[0],closest_cube[1],0.04,closest_cube[2]]]
    tower_closest_cube, closest_cube_index = cla.find_closest_cube(cubes_to_move, (closest_cube[0],closest_cube[1]), cubes_to_move_indexes)
    pick_poses.append([tower_closest_cube[0],tower_closest_cube[1],0.04,tower_closest_cube[2]])
    cubes_to_move_indexes.remove(closest_cube_index)
    cubes_to_move.remove(tower_closest_cube)
    tower_closest_cube, closest_cube_index = cla.find_closest_cube(cubes_to_move, (closest_cube[0],closest_cube[1]), cubes_to_move_indexes)
    pick_poses.append([tower_closest_cube[0],tower_closest_cube[1],0.04,tower_closest_cube[2]])

    print("=========== Place positions:",cubes_tower_pos)
    print("=========== Pick positions:", pick_poses)


    for i in range(1,3):
        print("pick_orientation:",pick_poses[i][2])
        cla.stack(cubes_tower_pos[i],[pick_poses[i][0],pick_poses[i][1],0.0225],pick_poses[i][3])

    cla.plan_and_move.move_standard_pose()
    """