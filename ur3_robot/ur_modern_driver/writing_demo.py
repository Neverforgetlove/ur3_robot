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
import json
import cmath
import math
from geometry_msgs.msg import PoseStamped,Pose,PointStamped
from sensor_msgs.msg import JointState

class writing_demo:
    def __init__(self):

        rospy.init_node('writing_demo',anonymous=True)

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

        self.waypoints = []
    
    def get_write_point(self,target,change_size):


        #0.0 0.0 -90 -90 -90 180
        self.arm.set_named_target("write")
        self.arm.go()
        rospy.sleep(1.0)

        target_len = len(target)

        start_point = self.arm.get_current_pose().pose

        start_pointX = start_point.position.x
        start_pointY = start_point.position.y
        start_pointZ = start_point.position.z

        for i in range(target_len):
            
            filename = '/home/cjw/PycharmProjects/writting/traj_test/'+target[i]+'.txt'
            print(target[i])
          
            fraction = 0.0
            max_fraction = 0.0

            write_point = self.arm.get_current_pose().pose  #笔长： 0.08
            
            if i > 0:
                self.waypoints.append(copy.deepcopy(write_point))
                write_point.position.y -= 0.08 #start_ponitY - start_pointY * (i)
                write_point.position.z = start_pointZ
                self.waypoints.append(copy.deepcopy(write_point))

                # write_point.position.x += 0.02
                # self.waypoints.append(copy.deepcopy(write_point))
            else:
                self.waypoints.append(copy.deepcopy(write_point))
                write_point.position.x += 0.10
                self.waypoints.append(copy.deepcopy(write_point))
            with open(filename,'r') as file_to_read:
                list = []

                while True:
                    lines = file_to_read.readline()
                    if not lines:
                        break
                    item = [j for j in lines.split(",")]
                    point_x = json.loads(item[0])
                    point_y = json.loads(item[1])
                    if point_x != 0.0 and point_y != 0.0:

                        #在opencv图像中的像素起始为（0,0）
                        #以末端点为起始点应加上末端实际偏移
                        #opencv中图像缩小5000得到实际的点
                        #缩小倍数自定义，要考虑机械臂的工作空间
                        point_x = (600 - point_x) / (6*1000.0) + write_point.position.y - 0.02
                        point_y = (600 - point_y) / (6*1000.0) + write_point.position.z - 0.05
                    list.append([point_x,point_y])
                
                # print(list)
                # self.waypoints.append(copy.deepcopy(write_point))
 
                print("!------------------!")
                for k in range(0,len(list)-1,1):

                    if list[k][0]==0 and list[k][1] == 0:
                        if i == 0:
                            write_point.position.x -= 0.02
                            self.waypoints.append(copy.deepcopy(write_point))
                        
                        k += 1
                        if( (list[k][0] != 0.0 and list[k][1] != 0.0) and (list[k+1][0] == 0.0 and list[k+1][1] == 0.0)):
                            print("Only one point , Pass!!!!")
                            k += 2
                            # continue
                  
                        write_point.position.y = list[k][0]
                        write_point.position.z = list[k][1]

                        self.waypoints.append(copy.deepcopy(write_point))

                        write_point.position.x += 0.02
                        self.waypoints.append(copy.deepcopy(write_point))
                    else:

                        write_point.position.y = list[k][0]
                        write_point.position.z = list[k][1]
                        
                        self.waypoints.append(copy.deepcopy(write_point))
            write_point.position.x -= 0.02
            self.waypoints.append(copy.deepcopy(write_point))     
            # print(self.waypoints)
            for l in range(0,100):

                (self.fllow_plan, fraction) = self.arm.compute_cartesian_path(
                                        self.waypoints,   # self.waypoints to follow
                                        0.001,        # eef_step
                                        0.0)         # jump_threshold
                if fraction > max_fraction:
                    max_fraction = fraction
                    self.max_fraction_plan = self.fllow_plan

                if fraction == 1.0:
                    break
            
            print(max_fraction)

            if max_fraction == 1.0:
                self.arm.execute(self.max_fraction_plan,wait = True)
                # self.arm.plan(self.max_fraction_plan)
                del self.waypoints[:]

                # return True
            else:
                
                del self.waypoints[:]
        
            rospy.sleep(3.0)
            # self.arm.set_named_target("write")
            # self.arm.go()
            # rospy.sleep(1.0)


if __name__ == "__main__":
    test = writing_demo()
    write_target = '慧'
    test.get_write_point(write_target.decode('utf-8'),6)
    
