import rospy
import moveit_commander
from pick_and_place_module.eef_control import MoveGroupControl
from pick_and_place_module.grasping import Gripper
from math import pi

class PlanAndMove:
    def __init__(self):
        self.pick_pose = None
        self.place_pose = None
        self.gripper_pose = None
        self.moveit_control = MoveGroupControl()
        self.gripper = Gripper()

    def setPickPose(self, x, y, z, qx, qy, qz, qw):
        self.pick_pose = [x, y, z, qx, qy, qz, qw]

    def setPlacePose(self, x, y, z, qx, qy, qz, qw):
        self.place_pose = [x, y, z, qx, qy, qz, qw]
    
    def setGripperPose(self, finger1, finger2):
        self.gripper_pose = [finger1, finger2]

    def execute_pick(self):
        move_group = self.moveit_control

    def execute_place(self):
        move_group = self.moveit_control

    def move_standard_pose(self):
        # Moves to a standard fixed joint position

