import rospy
import moveit_commander
from pick_and_place_module.eef_control import MoveGroupControl
from pick_and_place_module.grasping import Gripper
from math import pi

import moveit_msgs.msg
import geometry_msgs.msg

class PlanAndMove:
    def __init__(self):
        self.pick_pose = None
        self.place_pose = None
        self.gripper_pose = None
        self.moveit_control = MoveGroupControl()
        self.gripper = Gripper()

    def setPickPose(self, x, y, z, qx, qy, qz, qw):
        self.pick_pose = [x, y, z+0.11, qx, qy, qz, qw]
        self.after_pick_pose = [x, y, z+0.3, qx, qy, qz, qw]

    def setPlacePose(self, x, y, z, qx, qy, qz, qw):
        self.place_pose = [x, y, z+0.1, qx, qy, qz, qw]
        self.before_place_pose = [x, y, z+0.3, qx, qy, qz, qw]

    def execute_pick(self):
        move_group = self.moveit_control

        # Open gripper
        self.gripper.move(0.04, 0.04)
        rospy.sleep(2)

        # Move end effector up so that it doesn't stumble on other cubes
        move_group.go_to_pose_q_goal(*self.after_pick_pose)
        rospy.sleep(2)

        # Go to desired pose
        move_group.go_to_pose_q_goal(*self.pick_pose)
        rospy.sleep(2)

        # Close gripper
        result_grasp = self.gripper.grasp()
        rospy.sleep(2)

        if result_grasp == True:
            # Move end effector up so that it doesn't stumble on other cubes
            move_group.go_to_pose_q_goal(*self.after_pick_pose)
            rospy.sleep(2)

        return result_grasp
        

    def execute_place(self):
        move_group = self.moveit_control

        # Move end effector up so that it doesn't stumble on other cubes
        move_group.go_to_pose_q_goal(*self.before_place_pose)
        rospy.sleep(2)

        # Go to desired pose
        move_group.go_to_pose_q_goal(*self.place_pose)
        rospy.sleep(2)

        # Open gripper
        self.gripper.move(0.04, 0.04)
        rospy.sleep(2)

    def move_standard_pose(self):
        # Moves to standard fixed joint position
        move_group = self.moveit_control
        """
        # Other standard pose for camera without tilted angle
        j1 = 0.00012800795074863203
        j2 = -0.7840221891322567
        j3 = -2.3187151373171844e-05
        j4 = -2.3586997434553547
        j5 = -2.0424443644806445e-05
        j6 = 1.5713909927841874
        j7 = 0.785411093636176
        """
        '''
        j1 = 0.004425608632595956 
        j2 = -0.1776332457239861
        j3 = -0.04997807565715949
        j4 = -1.825997224179561
        j5 = -0.0032382293592747883
        j6 = 1.6928230071740233
        j7 = 0.783730496518573
        '''

        move_group = self.moveit_control

        joints = [-0.004206980856824341, -0.17810793743406797, -0.000250905855610183, -2.108358197583551, -0.0037260432143549826, 1.9190163774086315, 0.8117410393014712]

        move_group.go_to_joint_state(*joints)


        #move_group.go_to_joint_state(j1,j2,j3,j4,j5,j6,j7)

    def move_left_pose(self):
        # Moves to standard fixed joint position
        move_group = self.moveit_control

        joints = [0.1544528757042178, -0.14394418814928905, 0.015648645433615306, -1.8178459083692888, -0.15541488663028036, 1.7155173146032399, 1.9887875228837737]

        move_group.go_to_joint_state(*joints)

    def move_right_pose(self):
        # Moves to standard fixed joint position
        move_group = self.moveit_control

        joints = [0.004715505642791427, -0.18091970730337859, -0.22904179133901187, -1.8254551566355834, -0.003738068515543526, 1.7271905250814792, -0.07804289483358255]

        move_group.go_to_joint_state(*joints)

    def get_joints(self):
        move_group = self.moveit_control
        joints = move_group.get_current_joint_states()
        return joints

    



    

