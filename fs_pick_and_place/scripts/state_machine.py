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
    smach.State.__init__(self, outcomes=['tower_identified','no_tower_identified'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state SCAN')

    outcome = input("Enter outcome:")
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


# Define state OCCUPIED_GRID
class OccupiedGrid(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['grid_generated'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state OCCUPIED_GRID')

    outcome = input("Enter outcome:")
    return outcome

# Define state TRY_TOWER_FIXED_CUBE
class TryTowerFixedCube(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['tower_plan_success','tower_impossible','tower_fixed_cube_fail'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state TRY_TOWER_FIXED_CUBE')

    outcome = input("Enter outcome:")
    return outcome

# Define state TRY_TOWER_FREE_SPACE
class TryTowerFreeSpace(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['tower_plan_success','fail'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state TRY_TOWER_FREE_SPACE')

    outcome = input("Enter outcome:")
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

    # Create a sub SMACH state machine (for planning the tower)
    sm_sub_plan = smach.StateMachine(outcomes=['fail','tower_plan_success'])

    with sm_sub_plan:

      # Add states to container
      smach.StateMachine.add('OCCUPIED_GRID',OccupiedGrid(tower), transitions={'grid_generated':"TRY_TOWER_FIXED_CUBE"})
      smach.StateMachine.add('TRY_TOWER_FIXED_CUBE',TryTowerFixedCube(tower), transitions={'tower_impossible':'TRY_TOWER_FIXED_CUBE','tower_plan_success':'tower_plan_success','tower_fixed_cube_fail':"TRY_TOWER_FREE_SPACE"})
      smach.StateMachine.add('TRY_TOWER_FREE_SPACE',TryTowerFreeSpace(tower), transitions={'fail':'fail', 'tower_plan_success':'tower_plan_success'})
    
    smach.StateMachine.add('PLAN_TOWER', sm_sub_plan, transitions={'tower_plan_success':"PICK_AND_PLACE",'fail':'fail'})

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
