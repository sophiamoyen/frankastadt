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
    self.tower.plan_and_move.move_standard_pose()
    return 'scan_success'
 
# Define state PICK_PLACE
class PickPlace(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['tower_built'], input_keys=['cubes_tower_pos','pick_positions'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)

    for i in range(1,3):
        self.tower.stack(userdata.cubes_tower_pos[i],userdata.pick_positions[i])


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

    free_positions, impossible_positions = self.tower.find_free_space(cube_positions, 
                                                               cube_size, 
                                                               lim_x, 
                                                               lim_y, 
                                                               safety_distance)

    self.tower.plot_free_space(free_positions, 
                        impossible_positions, 
                        cube_size, 
                        cube_positions, 
                        lim_x, 
                        lim_y, 
                        safety_distance,
                        cube_indexes)

    closest_cube, closest_cube_index = self.tower.find_closest_cube_origin(cube_positions, desired_center, cube_indexes)
    print("========== Removed cube_{} from occupied space".format(closest_cube_index))


    cubes_to_move = cube_positions.copy()
    cubes_to_move_indexes = cube_indexes.copy()
    cubes_to_move_indexes.remove(closest_cube_index)
    cubes_to_move.remove(closest_cube)

    free_positions, impossible_positions = self.tower.find_free_space(cubes_to_move, 
                                                               cube_size, 
                                                               lim_x, 
                                                               lim_y, 
                                                               safety_distance)

    self.tower.plot_free_space(free_positions, 
                        impossible_positions, 
                        cube_size, 
                        cubes_to_move, 
                        lim_x, 
                        lim_y, 
                        safety_distance,
                        cubes_to_move_indexes)
    
    print("================= Generating tower strucutre with 3 cubes with vertical orientation starting from position of cube_{}:".format(closest_cube_index), closest_cube)
    cubes_tower_pos = self.tower.creates_tower3_structure(closest_cube, orientation="vertical")
    print("================= Tower strucure generated:",cubes_tower_pos)
    placement_possible = self.tower.check_possible_tower_place(cubes_tower_pos, impossible_positions, safety_distance)
    print("================= Placement possible:",placement_possible)
    
    if placement_possible == False:
        sys.exit(1)


    self.tower.plot_tower_place(free_positions, impossible_positions, cube_size, cubes_to_move, 
                         lim_x, lim_y, safety_distance, cubes_to_move_indexes, cubes_tower_pos)

    # Getting Pick  SIMULATION
    pick_positions = [[*closest_cube,rospy.get_param("cube_0_z")]]
    tower_closest_cube, closest_cube_index = self.tower.find_closest_cube_origin(cubes_to_move, closest_cube, cubes_to_move_indexes)
    pick_positions.append([*tower_closest_cube,rospy.get_param("cube_0_z")])
    cubes_to_move_indexes.remove(closest_cube_index)
    cubes_to_move.remove(tower_closest_cube)
    tower_closest_cube, closest_cube_index = self.tower.find_closest_cube_origin(cubes_to_move, closest_cube, cubes_to_move_indexes)
    pick_positions.append([*tower_closest_cube,rospy.get_param("cube_0_z")])

    print("=========== Place positions:",cubes_tower_pos)
    print("=========== Pick positions:", pick_positions)

    userdata.pick_positions = pick_positions
    userdata.cubes_tower_pos = cubes_tower_pos

    return 'tower_planned'

 
# Main method
def main():
 

  tower = Tower()
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