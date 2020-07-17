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
from geometry_msgs.msg import PoseStamped,Pose,PointStamped
from sensor_msgs.msg import JointState


class Fllow_ArCode:
    def __init__(self):

        rospy.init_node('Fllow_ARCode',anonymous=True)

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

        self.fllow_plan = self.arm.plan()
        self.max_fraction_plan = self.arm.plan()
        
        self.count = 0  #跟随次数
        self.fail_count = 0  #跟随失败次数

        self.listener = tf.TransformListener()

        #末端位姿
        self.x = 0
        self.y = 0
        self.z = 0
        self.o_x = 0
        self.o_y = 0
        self.o_z = 0
        self.z_w = 0

        #QR码位姿
        self.start_pose = PoseStamped()
        self.first_time = True

        #ar码相对移动距离
        self.ar_x = 0
        self.ar_y = 0
        self.ar_z = 0

        #规划失败累加位移差 
        self.Accumulate_x = 0
        self.Accumulate_y = 0
        self.Accumulate_z = 0

        #开始相对距离
        self.relative_distance = 0
        #跟随相对距离
        self.move_rel_distance = 0
        #路径点
        self.waypoints = []

    def start_fllow(self):
        self.Listen_ARCode()


    def Listen_ARCode(self):
        
        rospy.Subscriber("/aruco_single/pose",PoseStamped,self.ar_pose,queue_size=3)
        rospy.spin()


    def fllow_cartesian_path(self,move_x,move_y,move_z):           

        #当前位置为第一个点
        self.waypoints.append(self.arm.get_current_pose().pose)
        wpose = Pose()
        wpose1 = Pose()
        #路径点成功率
        fraction = 0.0

        max_fraction = 0.0
        
        wpose.position = self.waypoints[0].position 
        wpose.orientation = self.waypoints[0].orientation
        wpose1.orientation = self.waypoints[0].orientation
        
        print(wpose)

        #添加路径点
        print("位移量")
        print(move_x,move_y,move_z)
        for i in range(0,3):
            wpose1.position.x =  wpose.position.x + (move_x/3)*(i+1)
       
            wpose1.position.y =  wpose.position.y + (move_y/3)*(i+1)
       
            wpose1.position.z =  wpose.position.z + (move_z/3)*(i+1)


            # print(wpose1.position.x,wpose1.position.y,wpose1.position.z)
            self.waypoints.append(copy.deepcopy(wpose1))
        
        # print(self.waypoints)
        #规划100次，全部点规划成功fraction=1跳出，执行
        for i in range(0,100):
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

        #清除路径点
        # del self.waypoints[:]
        # wpose = 0

        #大于80%执行
        if max_fraction >= 0.8:
            self.arm.execute(self.max_fraction_plan,wait = True)
            del self.waypoints[:]
            return True
        else:
            #print("Motion Msg param [barm] = FALSE!")
            del self.waypoints[:]
            return False

    def ar_pose(self,data):

        print("开始跟随")

        #相机识别QR码的位置
        self.x = data.pose.position.x   
        self.y = data.pose.position.y
        self.z = data.pose.position.z
        self.ox = data.pose.orientation.x
        self.oy = data.pose.orientation.y
        self.oz = data.pose.orientation.z
        self.ow = data.pose.orientation.w

        #位置转换到base_link
        point_=PointStamped()               
        point_.header.frame_id="camera_link"
    
        point_.header.stamp=rospy.Time()
        point_.point.x= self.x
        point_.point.y= self.y
        point_.point.z= self.z
    
        result_=self.listener.transformPoint("base_link",point_)

        first_pose = self.arm.get_current_pose().pose
        
        #保存第一个QR码的开始位置
        if(self.first_time == True):      
            self.start_pose.pose.position.x = result_.point.x
            self.start_pose.pose.position.y = result_.point.y
            self.start_pose.pose.position.z = result_.point.z
            self.relative_distance = cmath.sqrt((first_pose.position.x - result_.point.x)**2 
            +(first_pose.position.y - result_.point.y)**2 + (first_pose.position.z - result_.point.z)**2)
            self.first_time = False
        #相对开始位置的移动量  相对距离   
        else:                             
            self.ar_x = result_.point.x - self.start_pose.pose.position.x
            self.ar_y = result_.point.y - self.start_pose.pose.position.y
            self.ar_z = result_.point.z - self.start_pose.pose.position.z
            self.move_rel_distance = cmath.sqrt((first_pose.position.x - result_.point.x)**2 
            +(first_pose.position.y - result_.point.y)**2 + (first_pose.position.z - result_.point.z)**2)

            print(self.ar_x,self.ar_y,self.ar_z)
            print("---------------------------------------------")
            print(self.move_rel_distance)

            print("---------------------------------------------")

            #相对位置移动距离大于0.03 移动量大于0.01，开始跟随 
            # 
            if(abs(self.relative_distance - self.move_rel_distance)>=0.03 or abs(self.ar_x)>0.01 or abs(self.ar_y)>0.01 or abs(self.ar_z)>0.01):

                self.count += 1     

                res = self.fllow_cartesian_path(self.ar_x,self.ar_y,self.ar_z)

                #跟随成功，更新QR码移动前位置
                #跟随失败，累加QR码移动量
                #有一次成功就清空
                if(res == True):
                    result = "成功"
                    self.start_pose.pose.position.x = result_.point.x #+ self.ar_x
                    self.start_pose.pose.position.y = result_.point.y #+ self.ar_y
                    self.start_pose.pose.position.z = result_.point.z #+ self.ar_z
                    if(self.fail_count):
                        self.fail_count = 0
                        self.Accumulate_x = 0
                        self.Accumulate_y = 0
                        self.Accumulate_z = 0
                else:
                    result = "失败"
                    self.fail_count += 1 
                    if(self.Accumulate_x != self.ar_x or self.Accumulate_y != self.ar_y or self.Accumulate_z != self.ar_z):
                        self.Accumulate_x += self.ar_x
                        self.Accumulate_y += self.ar_y
                        self.Accumulate_z += self.ar_z
                    print("累加量")
                    print(self.Accumulate_x, self.Accumulate_y, self.Accumulate_z)

                print("第{0}次跟随{1}".format(self.count,result))
                rospy.sleep(0.5)
                print("------------------------------------------------")
                self.ar_x = self.ar_y = self.ar_z = 0.0
            else:
                print("catch Qr position")
        

if __name__ == "__main__":
    test = Fllow_ArCode()
    test.start_fllow()
        
