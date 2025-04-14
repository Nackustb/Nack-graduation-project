#! /usr/bin/env python3.8
# -*- coding: utf-8 -*-

# import sys
# sys.path.append("/home/yan/WS/ur_arms/")
import rospy
from enum import Enum
from std_srvs.srv import SetBool
from std_msgs.msg import Float64
from yolov5_ros_msgs.msg import BoundingBoxes,BoundingBox

class ArmStage(Enum):
    IDLE = 0
    UpArm = 1
    BendsArm = 2
    StraightenArm = 3


class armControl:
    def __init__(self):
        rospy.init_node("arm_control_node")
    
        self.bboxes_sub = rospy.Subscriber(
            "/yolov5/BoundingBoxes",BoundingBoxes,self.bboxesCallback)
        self.srv_nav_start = rospy.Service(
            '/mov_start', SetBool, self.mov_start)
        self.arm_shoulder_pan_pub = rospy.Publisher(
            "/ur5_arm/shoulder_pan_joint_position_controller/command",Float64,queue_size=1000)
        self.shoulder_lift_pub = rospy.Publisher(
            "/ur5_arm/shoulder_lift_joint_position_controller/command",Float64,queue_size=1000)
        self.elbow_pub = rospy.Publisher(
            "/ur5_arm/elbow_joint_position_controller/command",Float64,queue_size=1000)
        self.wrist_1_pub = rospy.Publisher(
            "/ur5_arm/wrist_1_joint_position_controller/command",Float64,queue_size=1000)
        self.wrist_2_pub = rospy.Publisher(
            "/ur5_arm/wrist_2_joint_position_controller/command",Float64,queue_size=1000)
        self.wrist_3_pub = rospy.Publisher(
            "/ur5_arm/wrist_3_joint_position_controller/command",Float64,queue_size=1000)
        
        self.arm_shoulder_pan_position = Float64()
        self.shoulder_lift_position = Float64()
        self.elbow_position = Float64()        
        self.wrist_1_position = Float64()
        self.wrist_2_position = Float64()
        self.wrist_3_position = Float64()
        self.is_mov_start = False
        self.arm_stage = ArmStage.IDLE

        self.arm_shoulder_pan_position.data = rospy.get_param("~arm_shoulder_pan_position",3.14)
        self.shoulder_lift_position.data = rospy.get_param("~shoulder_lift_position",0.0)
        self.elbow_position.data = rospy.get_param("~elbow_position",0.0)
        self.wrist_1_position.data = rospy.get_param("~wrist_1_position",3.14)
        self.wrist_2_position.data = rospy.get_param("~wrist_2_position",0.0)
        self.wrist_3_position.data = rospy.get_param("~wrist_3_position",0.0)
    
    def mov_start(self, req):
        if not self.is_mov_start:
            if req.data:
                print("Action Stations!!!")
                self.is_mov_start = True
                return True, "Start move"
            else:
                print("Ignoring request as the \"data\" field is set to false")
                return False, "False received"
        else:
            if req.data:
                print("Starting failed: already running")
                return False, "Already running"
            else:
                return False, "doing nothing" 

    def bboxesCallback(self,msg):
        arm_status_temp = msg.bounding_boxes[0]
        arm_status = arm_status_temp.Class

        print(arm_status)
        if arm_status == 1: 
            self.arm_stage = ArmStage.BendsArm
            self.arm_shoulder_pan_position.data = 0.0
            self.shoulder_lift_position.data = 1.0
            self.elbow_position.data = -1.57
            self.wrist_1_position.data = 0.0
            self.wrist_2_position.data = 0.0
            self.wrist_3_position.data = 0.0
        elif arm_status == 2:
            self.arm_stage = ArmStage.StraightenArm
            self.arm_shoulder_pan_position.data = 0.0
            self.shoulder_lift_position.data = 0.8
            self.elbow_position.data = 0.0
            self.wrist_1_position.data = 0.0
            self.wrist_2_position.data = 0.0
            self.wrist_3_position.data = 0.0
        elif arm_status == 3:
            self.arm_stage = ArmStage.UpArm
            self.arm_shoulder_pan_position.data = 0.0
            self.shoulder_lift_position.data = 0.0
            self.elbow_position.data = 0.0
            self.wrist_1_position.data = 0.0
            self.wrist_2_position.data = 0.0
            self.wrist_3_position.data = 0.0
        else:
            self.arm_stage = ArmStage.IDLE
    

    def flow(self):
        if self.is_mov_start == True:
            if self.arm_stage == ArmStage.BendsArm:
                self.arm_shoulder_pan_pub.publish(self.arm_shoulder_pan_position)
                self.shoulder_lift_pub.publish(self.shoulder_lift_position)
                self.elbow_pub.publish(self.elbow_position)
                self.wrist_1_pub.publish(self.wrist_1_position)
                self.wrist_2_pub.publish(self.wrist_2_position)
                self.wrist_3_pub.publish(self.wrist_3_position)
                print("Now arm has been bended!")
            if self.arm_stage == ArmStage.StraightenArm:
                self.arm_shoulder_pan_pub.publish(self.arm_shoulder_pan_position)
                self.shoulder_lift_pub.publish(self.shoulder_lift_position)
                self.elbow_pub.publish(self.elbow_position)
                self.wrist_1_pub.publish(self.wrist_1_position)
                self.wrist_2_pub.publish(self.wrist_2_position)
                self.wrist_3_pub.publish(self.wrist_3_position)
                print("Now arm has been straightened!")
            if self.arm_stage == ArmStage.UpArm:
                self.arm_shoulder_pan_pub.publish(self.arm_shoulder_pan_position)
                self.shoulder_lift_pub.publish(self.shoulder_lift_position)
                self.elbow_pub.publish(self.elbow_position)
                self.wrist_1_pub.publish(self.wrist_1_position)
                self.wrist_2_pub.publish(self.wrist_2_position)
                self.wrist_3_pub.publish(self.wrist_3_position)
                print("Now arm has been up!")
            if self.arm_stage == ArmStage.IDLE:
                print("Nothing is done!")
            
    
    def run(self):
        rate = rospy.Rate(20)
        self.flow()
        while not rospy.is_shutdown():
            self.flow()
            rate.sleep()    

if __name__ == "__main__":
    try:
        arm_control = armControl()
        arm_control.run()

    except rospy.ROSInterruptException:
        pass
