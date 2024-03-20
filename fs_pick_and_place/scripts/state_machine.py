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

"""
-----------------------------------------------------------
States Definition
-----------------------------------------------------------
"""
 

# Define state INIT
class Init(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['init_success'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
    rospy.loginfo('Executing state INIT')

    # Create collision scene
    self.tower.collision_scene()
    # Goes to standard pose
    self.tower.plan_and_move.move_standard_pose()
    return 'init_success'

# Define state SCAN
class Scan(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['tower_identified','no_tower_identified'],output_keys=['cubes_poses','cubes_ids','cubes_yaws','tower_state'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
    rospy.loginfo('Executing state SCAN')

    # Getting number of detected cubes
    num_cubes = int(rospy.get_param("num_cubes"))

    # Getting the poses of the detected cubes in a list
    cubes_poses, cubes_ids, cubes_yaws = self.tower.get_detected_cubes(num_cubes)

    # EDIT HERE TOWER STATE
    tower_state = input("Enter outcome:")

    # Outputs data flow for the state machine
    userdata.cubes_poses = cubes_poses
    userdata.cubes_ids = cubes_ids
    userdata.cubes_yaws = cubes_yaws
    userdata.cubes_yaws = cubes_yaws
    userdata.tower_state = tower_state

    if tower_state=='0':
      outcome = 'no_tower_identified'

    else:
      outcome = 'tower_identified'
      
      
    return outcome


# Define state RESUME_TOWER
class ResumeTower(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['fail','tower_plan_success'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state RESUME_TOWER')

    outcome = input("Enter outcome:")
    return outcome



# Define state PLAN_TOWER
class PlanTower(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['fail','tower_plan_success'],input_keys=['cubes_poses','cubes_ids','cubes_yaws'],output_keys=['cubes_poses_place','cubes_poses_pick'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state PLAN_TOWER')
    # Checks cubes' poses and generated grid with occupied and free spots
    free_pos_general, occupied_pos_general = self.tower.find_free_space(userdata.cubes_poses)

    # Plots occupancy grid
    self.tower.plot_free_space(free_pos_general, 
                               occupied_pos_general, 
                               userdata.cubes_poses,
                               userdata.cubes_yaws,
                               userdata.cubes_ids)

    # Tries to build a tower around each cube first
    poses = userdata.cubes_poses.copy()
    ids = userdata.cubes_ids.copy()
    yaws = userdata.cubes_yaws.copy()

    for index in userdata.cubes_ids:
      free_pos, occupied_pos, closest_cube_id, closest_cube_pose, cubes_poses_except_base, cubes_yaws_except_base, cubes_ids_except_base = self.tower.clears_space_for_tower(poses, ids, yaws, tower_base=3)
        
      print("================= Generating tower strucutre starting from cube_{}:".format(closest_cube_id), closest_cube_pose)
      cubes_tower_pos = self.tower.creates_tower6_structure(closest_cube_pose, orientation="horizontal")
      print("================= Tower strucure generated:",cubes_tower_pos)
      placement_possible = self.tower.check_possible_tower_place(cubes_tower_pos, occupied_pos, tower_type=6)
      print("================= Placement possible:",placement_possible)
      
      if placement_possible == True:
        
        self.tower.plot_tower_place(free_pos, occupied_pos, cubes_poses_except_base, cubes_yaws_except_base, cubes_ids_except_base, cubes_tower_pos, tower_type=6)

        # Getting Pick Poses
        poses = userdata.cubes_poses.copy()
        ids = userdata.cubes_ids.copy()

        pick_poses = []
        for i in range(len(userdata.cubes_poses)):
          tower_closest_cube, closest_cube_id = self.tower.find_closest_cube(poses, (closest_cube_pose[0],closest_cube_pose[1]), ids)
          pick_poses.append([tower_closest_cube[0],tower_closest_cube[1],0.04,tower_closest_cube[2]])

          ids.remove(close_cube_id)
          poses.remove(tower_closest_cube)


        print("=========== Place positions:",cubes_tower_pos)
        print("=========== Pick positions:", pick_poses)

        
        userdata.pcubes_poses_pick = pick_poses
        userdata.cubes_poses_place = cubes_tower_pos
        outcome = 'tower_plan_success'
        break

      ids.remove(close_cube_id)
      poses.remove(closest_cube_pose)
      yaws.pop(closest_cube_id)

    
    if placement_possible == False:
      print("================= Couldn't find free space to build tower =================")
      outcome = 'fail'

    return outcome


# Define state PRE_CHECK
class PreCheck(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['pre_check_success','scenario_changed'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state PRE_CHECK')

    outcome = input("Enter outcome:")
    return outcome

# Define state PICK_CUBE
class PickCube(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['pick_success','scenario_changed','pick_failed'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state PICK_CUBE')

    outcome = input("Enter outcome:")
    return outcome

# Define state PICK_CHECK
class PickCheck(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['pick_check_success','scenario_changed'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state PICK_CHECK')

    outcome = input("Enter outcome:")
    return outcome

# Define state RETURN_CUBE
class ReturnCube(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['cube_returned'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state RETURN_CUBE')

    outcome = input("Enter outcome:")
    return outcome

# Define state PLACE_AND_CHECK
class PlaceAndCheck(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['tower_built','scenario_changed','next_pick'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state PLACE_AND_CHECK')

    outcome = input("Enter outcome:")
    return outcome


 

"""
-----------------------------------------------------------
SM Implementation
-----------------------------------------------------------
"""
# Main method
def main():
 

  tower = Tower()
  # Create the top SMACH state machine container
  sm_top = smach.StateMachine(outcomes=['fail','tower_built'])
 
     
  # Open the top state machine container
  with sm_top:
     
    # Add states to the container, and specify the transitions between states
    smach.StateMachine.add('INIT', Init(tower), transitions={'init_success':'SCAN'})
    smach.StateMachine.add('SCAN', Scan(tower), transitions={'tower_identified':'RESUME_TOWER','no_tower_identified':"PLAN_TOWER"})
    smach.StateMachine.add('RESUME_TOWER', ResumeTower(tower), transitions={'tower_plan_success':'PICK_AND_PLACE','fail':'fail'})
    smach.StateMachine.add('PLAN_TOWER',PlanTower(tower), transitions={'fail':'fail', 'tower_plan_success':'PICK_AND_PLACE'})

    # Create a sub SMACH state machine (for building the tower)
    sm_sub_build = smach.StateMachine(outcomes=['scenario_changed','tower_built'])

    with sm_sub_build:

      # Add states to container
      smach.StateMachine.add('PRE_CHECK',PreCheck(tower), transitions={'pre_check_success':"PICK_CUBE",'scenario_changed':'scenario_changed'})
      smach.StateMachine.add('PICK_CUBE',PickCube(tower), transitions={'pick_failed':'PICK_CUBE','pick_success':'PICK_CHECK','scenario_changed':"scenario_changed"})
      smach.StateMachine.add('PICK_CHECK',PickCheck(tower), transitions={'pick_check_success':'PLACE_AND_CHECK', 'scenario_changed':'RETURN_CUBE'})
      smach.StateMachine.add('RETURN_CUBE',ReturnCube(tower), transitions={'cube_returned':'scenario_changed'})
      smach.StateMachine.add('PLACE_AND_CHECK',PlaceAndCheck(tower), transitions={'next_pick':'PICK_CUBE','tower_built':'tower_built', 'scenario_changed':'scenario_changed'})
    
    smach.StateMachine.add('PICK_AND_PLACE', sm_sub_build, transitions={'tower_built':"tower_built",'scenario_changed':'SCAN'})
 
  # View our state transitions using ROS by creating and starting the instrospection server
  sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
  sis.start()
   
  # Execute the state machine 
  outcome = sm_top.execute()
 
  # Wait for ctrl-c to stop the application
  rospy.spin()
  sis.stop()
 
if __name__ == '__main__':
  main()
