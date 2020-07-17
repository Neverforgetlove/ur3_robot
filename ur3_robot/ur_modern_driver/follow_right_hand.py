#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
from moveit_ros_planning_interface import _moveit_move_group_interface
import tf
import copy
import threading
import cmath
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped,Pose,PointStamped
from sensor_msgs.msg import JointState


class Fllow_ArCode:
    def __init__(self):

        rospy.init_node('Fllow_RightHand',anonymous=True)

        moveit_commander.roscpp_initialize(sys.argv)              

        self.arm = moveit_commander.MoveGroupCommander('ur3_arm') 
     

        self.end_effector_link = self.arm.get_end_effector_link()                  

        self.reference_frame = 'base_link'

        self.arm.allow_replanning(False)   

        #速度   加速度
        self.arm.set_max_velocity_scaling_factor(0.1)
        self.arm.set_max_acceleration_scaling_factor(0.05)

        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)

        self.arm.set_start_state_to_current_state()

        self.angle1 = 0.0
        self.angle2 = 0.0
        self.move_with_hand = False
        self.arm_ready = True


    def start_fllow(self):
        if self.arm_ready == True:
            arm_ready_angle = self.arm.get_current_joint_values()
            if arm_ready_angle[0] != 0.0 or arm_ready_angle[1] != 0.0:
                arm_ready_angle[0] = 0.0
                arm_ready_angle[1] = 0.0
                self.arm.set_joint_value_target(arm_ready_angle)
                plan1 = self.arm.plan()
                self.arm.go(wait=True)
                self.arm_ready = False

        self.Listen_ARCode()


    def Listen_ARCode(self):
        while True:
            rospy.Subscriber("/angle1",Float64,self.get_angle1,queue_size=1)
            rospy.Subscriber("/angle2",Float64,self.get_angle2,queue_size=1)
            rospy.sleep(1)
            self.send_angle()
        # rospy.spin()

    def get_angle1(self,data):
        # print(data)
        self.angle1 = data.data
        # print(self.angle1)


    
    def get_angle2(self,data):
        self.angle2 = data.data
        # print("angle2=")
        # print(self.angle2)


    def send_angle(self):

        arm_angle = self.arm.get_current_joint_values()

        print(abs(self.angle1 - arm_angle[0]*57.29578))
        print(abs(self.angle2 - arm_angle[1]*57.29578))
       
       
        if self.angle1 != 0.0 and abs(self.angle1 - arm_angle[0]*57.29578) > 15:
            arm_angle[0] = self.angle1 / 57.29578 - math.pi/2
            self.move_with_hand = True
         
        if self.angle2 != 0.0 and abs(self.angle2 - arm_angle[1]*57.29578) > 15:
            arm_angle[1] = math.pi/2 - self.angle2 / 57.29578
            self.move_with_hand = True
        
        if self.move_with_hand == True:
            self.arm.set_joint_value_target(arm_angle)
            plan2 = self.arm.plan()
            self.arm.go(wait=True)

            rospy.sleep(1)
   
if __name__ == "__main__":
    test = Fllow_ArCode()
    test.start_fllow()
        
