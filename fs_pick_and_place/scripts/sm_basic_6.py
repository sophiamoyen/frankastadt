#!/usr/bin/env python3

# Import the necessary libraries
import rospy # Python client library
import smach
import smach_ros
from time import sleep # Handle time
from tower import Tower
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
 
# Define state LOCKED
class Scan(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['scan_success'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state SCAN')
    self.tower.collision_scene()
    self.tower.plan_and_move.move_standard_pose()
    return 'scan_success'
 
# Define state PICK_PLACE
class PickPlace(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['tower_built'], input_keys=['cubes_tower_pos','pick_positions'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)

    n_detected_cubes = int(rospy.get_param("num_cubes"))
    for i in range(1,n_detected_cubes):
        self.tower.stack(userdata.cubes_tower_pos[i],[userdata.pick_positions[i][0],userdata.pick_positions[i][1],0.0225],userdata.pick_positions[i][3])


    self.tower.plan_and_move.move_standard_pose()
 
    rospy.loginfo('Executing state PICK PLACE')

    return 'tower_built'

# Define state PLAN_TOWER
class PlanTower(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['tower_planned'], output_keys=['cubes_tower_pos','pick_positions'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
 
    rospy.loginfo('Executing state PLAN TOWER')

    n_detected_cubes = int(rospy.get_param("num_cubes"))

    
    cube_poses = []
    cube_yaws = []
    cube_indexes = []
    for i in range(n_detected_cubes):
        pick_pose = [rospy.get_param("cube_{}_x".format(i)),
                     rospy.get_param("cube_{}_y".format(i)),
                     self.tower.convert_orientation(rospy.get_param("cube_{}_orient_z".format(i)))]
        cube_yaws.append(rospy.get_param("cube_{}_orient_z".format(i)))
        cube_poses.append(pick_pose)
        cube_indexes.append(i)

    free_positions, impossible_positions = self.tower.find_free_space(cube_poses)

    self.tower.plot_free_space(free_positions, 
                                impossible_positions, 
                                cube_poses,
                                cube_yaws,
                                cube_indexes)


    cubes_to_move = cube_poses.copy()
    cubes_to_move_indexes = cube_indexes.copy()

    cubes_except_base = cube_poses.copy()
    cubes_except_base_indexes = cube_indexes.copy()

    closest_cube_0, closest_cube_index_0 = self.tower.find_closest_cube(cubes_except_base, desired_center, cubes_except_base_indexes)
    print("========== Removed cube_{} from occupied space".format(closest_cube_index_0))

    cubes_to_move_indexes.remove(closest_cube_index_0)
    cubes_to_move.remove(closest_cube_0)
    cubes_except_base_indexes.remove(closest_cube_index_0)
    cubes_except_base.remove(closest_cube_0)

    closest_cube, closest_cube_index = self.tower.find_closest_cube(cubes_except_base, desired_center, cubes_except_base_indexes)
    print("========== Removed cube_{} from occupied space".format(closest_cube_index))

    cubes_except_base_indexes.remove(closest_cube_index)
    cubes_except_base.remove(closest_cube)

    closest_cube, closest_cube_index = self.tower.find_closest_cube(cubes_except_base, desired_center, cubes_except_base_indexes)
    print("========== Removed cube_{} from occupied space".format(closest_cube_index))

    cubes_except_base_indexes.remove(closest_cube_index)
    cubes_except_base.remove(closest_cube)

    free_positions, impossible_positions = self.tower.find_free_space(cubes_except_base)

    self.tower.plot_free_space(free_positions, 
                        impossible_positions, 
                        cubes_except_base, 
                        cube_yaws,
                        cubes_except_base_indexes)
    
    print("================= Generating tower strucutre with 3 cubes with vertical orientation starting from position of cube_{}:".format(closest_cube_index), closest_cube)
    cubes_tower_pos = self.tower.creates_tower6_structure(closest_cube_0, orientation="horizontal")
    print("================= Tower strucure generated:",cubes_tower_pos)
    placement_possible = self.tower.check_possible_tower_place(cubes_tower_pos, impossible_positions, tower_type=6)
    print("================= Placement possible:",placement_possible)
    
    if placement_possible == False:
        sys.exit(1)


    self.tower.plot_tower_place(free_positions, impossible_positions, cubes_except_base, cube_yaws, cubes_except_base_indexes, cubes_tower_pos, tower_type=6)

    # Getting Pick  SIMULATION
    pick_poses = [[closest_cube_0[0],closest_cube[1],0.04,closest_cube[2]]]

    for i in range(1,n_detected_cubes):
      tower_closest_cube, closest_cube_index = self.tower.find_closest_cube(cubes_to_move, (closest_cube_0[0],closest_cube_0[1]), cubes_to_move_indexes)
      pick_poses.append([tower_closest_cube[0],tower_closest_cube[1],0.04,tower_closest_cube[2]])
      cubes_to_move_indexes.remove(closest_cube_index)
      cubes_to_move.remove(tower_closest_cube)


    print("=========== Place positions:",cubes_tower_pos)
    print("=========== Pick positions:", pick_poses)

    
    userdata.pick_positions = pick_poses
    userdata.cubes_tower_pos = cubes_tower_pos

    return 'tower_planned'

 
# Main method
def main():
 

  tower = Tower(scenario="real")
  # Create a SMACH state machine container
  sm = smach.StateMachine(outcomes=['success','failed'])
 
     
  # Open the state machine container. A state machine container holds a number of states.
  with sm:
     
    # Add states to the container, and specify the transitions between states
    smach.StateMachine.add('SCAN', Scan(tower), transitions={'scan_success':'PLAN_TOWER'})
    smach.StateMachine.add('PLAN_TOWER', PlanTower(tower), transitions={'tower_planned':'PICK_PLACE'}, remapping={'cubes_tower_pos':'cubes_tower_pos','pick_positions':'pick_positions'})
    smach.StateMachine.add('PICK_PLACE', PickPlace(tower), transitions={'tower_built':'success'},remapping={'cubes_tower_pos':'cubes_tower_pos','pick_positions':'pick_positions'})
 
  # View our state transitions using ROS by creating and starting the instrospection server
  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()
   
  # Execute the state machine 
  outcome = sm.execute()
 
  # Wait for ctrl-c to stop the application
  rospy.spin()
  sis.stop()
 
if __name__ == '__main__':
  main()
