#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
from moveit_ros_planning_interface import _moveit_move_group_interface
import tf
import copy
import threading
import numpy as np
import cmath
import math
from geometry_msgs.msg import PoseStamped,Pose,PointStamped
from sensor_msgs.msg import JointState

class cricle_demo:
    def __init__(self):

        rospy.init_node('cricle_demo',anonymous=True)

        moveit_commander.roscpp_initialize(sys.argv)              

        self.arm = moveit_commander.MoveGroupCommander('ur3_arm') 
     

        self.end_effector_link = self.arm.get_end_effector_link()                  

        self.reference_frame = 'base_link'

        self.arm.allow_replanning(False)   

        #速度   加速度
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_acceleration_scaling_factor(0.5)

        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)

        self.arm.set_start_state_to_current_state()

        self.fllow_plan = self.arm.plan()
        self.max_fraction_plan = self.arm.plan()


    def draw_cricle(self):
        
        self.arm.set_named_target("up")
        self.arm.go()
        rospy.sleep(1.0)
        self.arm.clear_pose_targets()

        cricle_start_pose = PoseStamped()

        # cricle_start_pose.header.frame_id = self.end_effector_link

        cricle_start_pose.pose.orientation.x = 0.70711
        cricle_start_pose.pose.orientation.y = 0
        cricle_start_pose.pose.orientation.z = 0
        cricle_start_pose.pose.orientation.w = 0.70711

        cricle_start_pose.pose.position.x = 0.140859
        cricle_start_pose.pose.position.y = 0.36739
        cricle_start_pose.pose.position.z = 0.84716

        self.arm.set_pose_target(cricle_start_pose, self.end_effector_link)
        self.arm.go()
        rospy.sleep(1.0)
        self.arm.clear_pose_targets()

        print("----success------------")

        waypoints = []
        fraction = 0.0
        max_fraction = 0.0

        now_pose = self.arm.get_current_pose().pose
        waypoints.append(now_pose)

        centerA = now_pose.position.x
        centerB = now_pose.position.y
        radius = 0.15

        for i in np.arange(0.0,math.pi*2 ,0.01):
            now_pose.position.x = centerA + radius * math.cos(i)
            now_pose.position.y = centerB + radius * math.sin(i)
            waypoints.append(now_pose)

        for i in range(0,100):
            (self.fllow_plan, fraction) = self.arm.compute_cartesian_path(
                                    waypoints,   # self.waypoints to follow
                                    0.001,        # eef_step
                                    0.0)         # jump_threshold
            if fraction > max_fraction:
                max_fraction = fraction
                self.max_fraction_plan = self.fllow_plan

            if fraction == 1.0:
                break
        
        print(max_fraction)

        if max_fraction >= 0.85:
            self.arm.execute(self.max_fraction_plan,wait = True)
            del waypoints[:]
            # return True
        else:
            #print("Motion Msg param [barm] = FALSE!")
            del waypoints[:]
            # return False
        # self.arm.set_named_target("up")
        # self.arm.go()

        rospy.sleep(1.0)

if __name__ == "__main__":
    test = cricle_demo()
    test.draw_cricle()
    
