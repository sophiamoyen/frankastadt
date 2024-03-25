import rospy
from std_msgs.msg import Float64
from franka_gripper.msg import MoveActionGoal, GraspActionGoal,GraspActionResult, MoveActionResult
from control_msgs.msg import GripperCommandActionGoal

class Gripper:
    def __init__(self):
        self.gripper_pub = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=10)
        self.gripper_move_pub = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size=10)
        self.gripper_move_result_pub = rospy.Subscriber('/franka_gripper/move/result', MoveActionResult, self.move_callback)
        self.gripper_grasp_result_pub = rospy.Subscriber('/franka_gripper/grasp/result', GraspActionResult, self.grasp_callback)
        self.gripper_move_result = MoveActionResult()
        self.gripper_grasp_result = GraspActionResult()
        rospy.sleep(1)


    def move_callback(self, data):
        self.gripper_move_result = data

    def grasp_callback(self, data):
        self.gripper_grasp_result = data
    
    def move(self, finger1_y, finger2_y):
        
        gripper_data = MoveActionGoal()
        gripper_data.goal.width = finger1_y+finger2_y
        gripper_data.goal.speed = 0.1
        self.gripper_move_pub.publish(gripper_data)

        rospy.sleep(3)
        return self.gripper_move_result.result.success
        

    def grasp(self):
        
        gripper_data = GraspActionGoal()
        gripper_data.goal.width = 0.045
        gripper_data.goal.epsilon.inner = 0.1
        gripper_data.goal.epsilon.outer = 0.1
        gripper_data.goal.force = 10.0
        gripper_data.goal.speed = 0.1

        rospy.loginfo("Executing grasp Width:%f, Force:%f", gripper_data.goal.width, gripper_data.goal.force)
        
        self.gripper_pub.publish(gripper_data)
        rospy.sleep(3)
        return self.gripper_grasp_result.result.success


        
        
        

        
        
