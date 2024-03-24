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

    # Going to standard pose
    self.tower.plan_and_move.move_standard_pose()

    # Getting tower_state
    sleep(10)

    # Getting tower_state
    tower_state = int(rospy.get_param("pyramid_state"))
    center_pose_tower = (rospy.get_param("pyramid_x"),rospy.get_param("pyramid_y"))

    # Getting number of detected cubes
    num_cubes = int(rospy.get_param("num_cubes"))

    # Getting the poses of the detected cubes in a list
    cubes_poses, cubes_ids, cubes_yaws = self.tower.get_detected_cubes(num_cubes, center_pose_tower, tower_state)

    # Outputs data flow for the state machine
    userdata.cubes_poses = cubes_poses
    userdata.cubes_ids = cubes_ids
    userdata.cubes_yaws = cubes_yaws
    userdata.tower_state = tower_state

    if tower_state==1 or tower_state==0:
      outcome = 'no_tower_identified'

    else:
      outcome = 'tower_identified'
      
      
    return outcome


# Define state RESUME_TOWER
class ResumeTower(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['fail','tower_plan_success'],input_keys=['cubes_poses','cubes_ids','cubes_yaws','tower_state'],output_keys=['cubes_poses_place','cubes_poses_pick','tower_state'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state RESUME_TOWER')
    # Checks cubes' poses and generated grid with occupied and free spots
    free_pos, occupied_pos = self.tower.find_free_space(userdata.cubes_poses)

    # Plots occupancy grid
    self.tower.plot_free_space(free_pos, 
                               occupied_pos, 
                               userdata.cubes_poses,
                               userdata.cubes_yaws,
                               userdata.cubes_ids)

    # EDIT HERE GET FROM PERCEPTION (x,y)
    center_pose_tower = (rospy.get_param("pyramid_x"),rospy.get_param("pyramid_y"))

    print("================= Generating tower strucutre starting from {}:".format(center_pose_tower))
    cubes_tower_pos = self.tower.creates_tower6_structure(center_pose_tower, orientation="horizontal")
    print("================= Tower strucure generated:",cubes_tower_pos)
      
        
    self.tower.plot_tower_place(free_pos, occupied_pos, userdata.cubes_poses, userdata.cubes_yaws, userdata.cubes_ids, cubes_tower_pos, tower_type=6)

    # Getting Pick Poses
    poses = userdata.cubes_poses.copy()
    ids = userdata.cubes_ids.copy()

    pick_poses = []
    for i in range(6-userdata.tower_state):
      tower_closest_cube, closest_cube_id = self.tower.find_closest_cube(poses, (center_pose_tower[0],center_pose_tower[1]), ids)
      pick_poses.append([tower_closest_cube[0],tower_closest_cube[1],0.04,tower_closest_cube[2]])

      ids.remove(closest_cube_id)
      poses.remove(tower_closest_cube)


    pick_poses = pick_poses
    cubes_tower_pos = cubes_tower_pos[userdata.tower_state:]


    print("=========== Place positions:",cubes_tower_pos)
    print("=========== Pick positions:", pick_poses)

    userdata.cubes_poses_place = cubes_tower_pos
    userdata.cubes_poses_pick = pick_poses
    outcome = 'tower_plan_success'
    

    return outcome



# Define state PLAN_TOWER
class PlanTower(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['fail','tower_plan_success'],input_keys=['cubes_poses','cubes_ids','cubes_yaws','tower_state'],output_keys=['cubes_poses_place','cubes_poses_pick','tower_state'])
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
      cubes_tower_pos = self.tower.creates_tower6_structure(closest_cube_pose[:2], orientation="horizontal")
      print("================= Tower strucure generated:",cubes_tower_pos)
      placement_possible = self.tower.check_possible_tower_place(cubes_tower_pos, occupied_pos, tower_type=6)
      print("================= Placement possible:",placement_possible)
      
      if placement_possible == True:
        
        self.tower.plot_tower_place(free_pos, occupied_pos, cubes_poses_except_base, cubes_yaws_except_base, cubes_ids_except_base, cubes_tower_pos, tower_type=6)

        # Getting Pick Poses
        poses = userdata.cubes_poses.copy()
        ids = userdata.cubes_ids.copy()

        pick_poses = []
        for i in range(len(cubes_tower_pos)):
          tower_closest_cube, close_cube_id = self.tower.find_closest_cube(poses, (closest_cube_pose[0],closest_cube_pose[1]), ids)
          pick_poses.append([tower_closest_cube[0],tower_closest_cube[1],0.04,tower_closest_cube[2]])

          ids.remove(close_cube_id)
          poses.remove(tower_closest_cube)


        print("=========== Place positions:",cubes_tower_pos)
        print("=========== Pick positions:", pick_poses)

        
        print("Cubes Tower Pos:", cubes_tower_pos)
        userdata.cubes_poses_place = cubes_tower_pos
        userdata.cubes_poses_pick = pick_poses
        outcome = 'tower_plan_success'
        break

      else:
        cube_id = ids.index(closest_cube_id)
        ids.remove(closest_cube_id)
        poses.remove(closest_cube_pose)
        yaws.pop(cube_id)

    
    if placement_possible == False:
      print("================= Couldn't find free space to build tower =================")
      outcome = 'fail'

    return outcome


# Define state PRE_CHECK
class PreCheck(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['pre_check_success','scenario_changed'], input_keys=['cubes_poses_pick'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state PRE_CHECK')

    # Getting number of detected cubes
    num_cubes = int(rospy.get_param("num_cubes"))

    # Getting tower_state
    tower_state = int(rospy.get_param("pyramid_state"))
    center_pose_tower = (rospy.get_param("pyramid_x"),rospy.get_param("pyramid_y"))

    # Getting the poses of the detected cubes in a list
    cubes_poses, cubes_ids, cubes_yaws = self.tower.get_detected_cubes(num_cubes, center_pose_tower, tower_state)

    # Checks cubes' poses and generated grid with occupied and free spots
    free_pos_general, occupied_pos_general = self.tower.find_free_space(cubes_poses)

    # Plots occupancy grid
    self.tower.plot_free_space(free_pos_general, 
                               occupied_pos_general, 
                               cubes_poses,
                               cubes_yaws,
                               cubes_ids)

    # Checks if cubes moved their position
    cubes_match = self.tower.check_cubes_match(userdata.cubes_poses_pick,cubes_poses)

    if cubes_match == False:
      outcome = 'scenario_changed'

    else:
      # DO!!! Substitute the cubes poses from the new scan
      outcome = 'pre_check_success'

    return outcome

# Define state PICK_CUBE
class PickCube(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['pick_success','scenario_changed'],input_keys=['cubes_poses_pick'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state PICK_CUBE')

    # Initialize grasp success variable
    result_grasp = False
    picks_tried = 0

    while result_grasp == False and picks_tried < 3:
      # Set pick pose, gets first cube from the list
      pick_position = [userdata.cubes_poses_pick[0][0],userdata.cubes_poses_pick[0][1],0.0225]
      pick_orientation = userdata.cubes_poses_pick[0][3]
      self.tower.plan_and_move.setPickPose(*pick_position,*pick_orientation)

      # Execute pick
      result_grasp = self.tower.plan_and_move.execute_pick()
      picks_tried += 1

    if result_grasp == True:
      outcome = 'pick_success'
    
    else:
      outcome = 'scenario_changed'

    return outcome

# Define state PICK_CHECK
class PickCheck(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['pick_check_success','scenario_changed'],input_keys=['tower_state'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state PICK_CHECK')
    # Going to standard pose
    self.tower.plan_and_move.move_standard_pose()

    # Getting tower_state
    sleep(12)
    tower_state = int(rospy.get_param("pyramid_state"))

    if userdata.tower_state == tower_state or userdata.tower_state == tower_state + 1:
      outcome = 'pick_check_success'

    else:
      outcome = 'scenario_changed'

    return outcome

# Define state RETURN_CUBE
class ReturnCube(smach.State):

  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['cube_returned'],input_keys=['cubes_poses_pick'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
    
    rospy.loginfo('Executing state RETURN_CUBE')
    
    # Set place pose, gets first cube from the list
    place_position = [userdata.cubes_poses_pick[0][0],userdata.cubes_poses_pick[0][1],0.04]
    place_orientation = userdata.cubes_poses_pick[0][3]
    self.tower.plan_and_move.setPlacePose(*place_position,*place_orientation)

    # Execute place
    self.tower.plan_and_move.execute_place()

    outcome = 'cube_returned'
    return outcome

# Define state PLACE_AND_CHECK
class PlaceAndCheck(smach.State):
  def __init__(self, tower):
    smach.State.__init__(self, outcomes=['tower_built','scenario_changed','next_pick'], input_keys=['cubes_poses_place','cubes_poses_pick','tower_state'],output_keys=['cubes_poses_place','cubes_poses_pick','tower_state'])
    self.tower = tower

  def execute(self, userdata):
    sleep(1)
     
    rospy.loginfo('Executing state PLACE_AND_CHECK')

    # # Set place pose, gets first cube from the list
    place_orientation_horizontal = [0.4134695331936024,
                                    -0.9104340626144781,
                                    -0.012313752255495124,
                                    0.0010650151882551477]

    place_position = userdata.cubes_poses_place[0]
    print("Place position:", place_position)
    print("Set Place Pose:",[*place_position,*place_orientation_horizontal])
    self.tower.plan_and_move.setPlacePose(*place_position,*place_orientation_horizontal)
    # Execute place
    self.tower.plan_and_move.execute_place()

    # Checks if the tower looks right
    # Going to standard pose
    self.tower.plan_and_move.move_standard_pose()

    # Getting tower_state
    sleep(5)
    tower_state = int(rospy.get_param("pyramid_state"))

    if userdata.tower_state == tower_state or userdata.tower_state == tower_state+1 :
      if len(userdata.cubes_poses_pick) == 1:
        outcome = 'tower_built'

      else:
        outcome = 'next_pick'

        userdata.cubes_poses_place.pop(0)
        userdata.cubes_poses_pick.pop(0)
        userdata.tower_state = tower_state+1


    else:
      outcome = 'scenario_changed'
    
    return outcome

 

"""
-----------------------------------------------------------
SM Implementation
-----------------------------------------------------------
"""
# Main method
def main():
 

  tower = Tower()
  tower.plan_and_move.move_standard_pose()
  # Create the top SMACH state machine container
  sm_top = smach.StateMachine(outcomes=['fail','tower_built'])
 
     
  # Open the top state machine container
  with sm_top:
     
    # Add states to the container, and specify the transitions between states
    smach.StateMachine.add('INIT', Init(tower), transitions={'init_success':'SCAN'})
    smach.StateMachine.add('SCAN', Scan(tower), transitions={'tower_identified':'RESUME_TOWER','no_tower_identified':"PLAN_TOWER"}, remapping={'cubes_poses':'cubes_poses','cubes_ids':'cubes_ids','cubes_yaws':'cubes_yaws','tower_state':'tower_state'})
    smach.StateMachine.add('RESUME_TOWER', ResumeTower(tower), transitions={'tower_plan_success':'PICK_AND_PLACE','fail':'fail'}, remapping={'cubes_poses':'cubes_poses','cubes_ids':'cubes_ids','cubes_yaws':'cubes_yaws','tower_state':'tower_state'})
    smach.StateMachine.add('PLAN_TOWER',PlanTower(tower), transitions={'fail':'fail', 'tower_plan_success':'PICK_AND_PLACE'}, remapping={'cubes_poses':'cubes_poses','cubes_ids':'cubes_ids','cubes_yaws':'cubes_yaws','tower_state':'tower_state','cubes_poses_place':'cubes_poses_place'})

    # Create a sub SMACH state machine (for building the tower)
    sm_sub_build = smach.StateMachine(outcomes=['scenario_changed','tower_built'], input_keys=['cubes_poses_pick','cubes_poses_place','tower_state'])

    with sm_sub_build:

      # Add states to container
      smach.StateMachine.add('PRE_CHECK',PreCheck(tower), transitions={'pre_check_success':"PICK_CUBE",'scenario_changed':'scenario_changed'}, remapping={'cubes_poses_pick':'cubes_poses_pick'})
      smach.StateMachine.add('PICK_CUBE',PickCube(tower), transitions={'pick_success':'PICK_CHECK','scenario_changed':"scenario_changed"},remapping={'cubes_poses_pick':'cubes_poses_pick'})
      smach.StateMachine.add('PICK_CHECK',PickCheck(tower), transitions={'pick_check_success':'PLACE_AND_CHECK', 'scenario_changed':'RETURN_CUBE'},remapping={'tower_state':'tower_state'})
      smach.StateMachine.add('RETURN_CUBE',ReturnCube(tower), transitions={'cube_returned':'scenario_changed'},remapping={'cubes_poses_pick':'cubes_poses_pick'})
      smach.StateMachine.add('PLACE_AND_CHECK',PlaceAndCheck(tower), transitions={'next_pick':'PICK_CUBE','tower_built':'tower_built', 'scenario_changed':'scenario_changed'}, remapping={'cubes_poses_pick':'cubes_poses_pick','cubes_poses_place':'cubes_poses_place','tower_state':'tower_state'})
    
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
