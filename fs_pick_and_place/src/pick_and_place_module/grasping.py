import rospy
from std_msgs.msg import Float64
from franka_gripper.msg import MoveActionGoal, GraspActionGoal

class Gripper:
    def __init__(self):
        self.gripper_pub = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=10)
        self.gripper_move_pub = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size=10)
        
        rospy.sleep(1)
    
    def move(self, finger1_y, finger2_y):
        
        gripper_data = MoveActionGoal()
        gripper_data.goal.width = finger1_y+finger2_y
       
        gripper_data.goal.speed = 0.1
        self.gripper_move_pub.publish(gripper_data)
        

    def grasp(self, finger1_y, finger2_y):
        
        gripper_data = GraspActionGoal()
        gripper_data.goal.width = finger1_y+finger2_y
        gripper_data.goal.epsilon.inner = 0.1
        gripper_data.goal.epsilon.outer = 0.1
        gripper_data.goal.force = 5.0
        gripper_data.goal.speed = 0.1
        self.gripper_pub.publish(gripper_data)
        
        
        

        
        
