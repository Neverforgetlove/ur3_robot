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

        #QR码位姿
        self.start_pose = PoseStamped()
        self.start_calibration = False
        self.change_start_pose = True

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

        self.start_arm_pose = self.arm.get_current_pose().pose
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = self.reference_frame
        self.target_pose.pose.orientation = self.start_arm_pose.orientation

        self.ar_point = PoseStamped()
        self.ar_point.header.frame_id = "camera_link"

        self.follow_res = True

    def start_fllow(self):
        self.Listen_ARCode()


    def Listen_ARCode(self):
        
        rospy.Subscriber("/aruco_single/pose",PoseStamped,self.follow_ar_pose,queue_size=1)
        rospy.spin()

    
    def move_with_code(self,data):
        
        ar_res_pose = self.listener.transformPose("base_link",self.ar_point)

        self.target_pose.header.stamp = rospy.Time.now()  
        self.target_pose.pose.position.x = ar_res_pose.pose.position.x - 0.35
        self.target_pose.pose.position.y = ar_res_pose.pose.position.y - 0.02
        self.target_pose.pose.position.z = ar_res_pose.pose.position.z - 0.04

        print(self.target_pose)
        self.arm.set_pose_target(self.target_pose, self.end_effector_link)
        self.arm.plan()
        rospy.sleep(2.0)
        self.arm.clear_pose_targets()
        
    # def follow_ar_pose(self,data):

    #     if(self.follow_res):
    #         self.ar_point.header.stamp = rospy.Time()
    #         self.ar_point.pose.position.x = data.pose.position.x
    #         self.ar_point.pose.position.y = data.pose.position.y
    #         self.ar_point.pose.position.z = data.pose.position.z

    #     ar_res_pose = self.listener.transformPose("base_link",self.ar_point)
        
    #     ar_res_pose.pose.position.x -= 0.35
    #     ar_res_pose.pose.position.y += 0.02
    #     ar_res_pose.pose.position.z -= 0.03

    #     print(ar_res_pose)

    #     self.arm.set_pose_target(ar_res_pose, self.end_effector_link)

    #     self.arm.plan()
    #     self.count += 1
    #     print(" count ",self.count) 
    #  # 关闭并退出moveit
    #     #moveit_commander.roscpp_shutdown()
    #     #moveit_commander.os._exit(0)
    #     self.arm.clear_pose_targets()
    #     print("清除") 

    #     rospy.sleep(0.5)

    def follow_ar_pose(self,data):
        
        if(self.follow_res):
            self.ar_point.header.stamp = rospy.Time()
            self.ar_point.pose.position.x = data.pose.position.x
            self.ar_point.pose.position.y = data.pose.position.y
            self.ar_point.pose.position.z = data.pose.position.z

        ar_res_point = self.listener.transformPose("base_link",self.ar_point)
        # ar_res_orion = self.listener.transformOr
        arm_pose = self.arm.get_current_pose().pose
        print(ar_res_point)
        
        r = math.atan2(2 * (data.pose.orientation.w * data.pose.orientation.x + data.pose.orientation.y * data.pose.orientation.z), 
                    1 - 2 * (data.pose.orientation.x * data.pose.orientation.x + data.pose.orientation.y * data.pose.orientation.y))
        r = (r / math.pi) * 180

        p = math.asin(2 * (data.pose.orientation.w * data.pose.orientation.y - 
                            data.pose.orientation.z * data.pose.orientation.x))
        p = (p / math.pi) * 180

        y = math.atan2(2 * (data.pose.orientation.w * data.pose.orientation.z + data.pose.orientation.x * data.pose.orientation.y), 
                    1 - 2 * (data.pose.orientation.y * data.pose.orientation.y + data.pose.orientation.z * data.pose.orientation.z))
        y = (y / math.pi) * 180

        ar_arm_x = ar_res_point.pose.position.x - (arm_pose.position.x+0.35)
        ar_arm_y = ar_res_point.pose.position.y - (arm_pose.position.y+0.01)  
        ar_arm_z = ar_res_point.pose.position.z - (arm_pose.position.z-0.05)  

        print("绕x轴旋转角度: {}".format(r))
        print("绕y轴旋转角度: {}".format(p))
        print("绕z轴旋转角度: {}\n".format(y))

        # if (abs(ar_arm_x) >= 0.365):
        #     arm_move_x = ar_arm_x - 0.35

        if(abs(ar_arm_x)>0.015 or abs(ar_arm_y)>0.015 or abs(ar_arm_z>0.015)):
            print("开始跟随")
            self.count += 1
            self.follow_res = self.follow_cartesian_path(data.pose.orientation, ar_arm_x, ar_arm_y, ar_arm_z)
            if(self.follow_res):
                res = "成功"
                print("第{0}次跟随{1}".format(self.count,res))
                rospy.sleep(0.5)
            else:
                res = "失败"
                print("第{0}次跟随{1}".format(self.count,res))
                self.fail_count += 1
                
                if(self.fail_count>3):
                    self.follow_res = True
                    self.fail_count = 0
                rospy.sleep(0.5)
        else:
            print("catch Qr position")


    # def ar_pose(self,data):


    #     #位置转换到base_link
    #     point_=PointStamped()               
    #     point_.header.frame_id="camera_link"
    
    #     point_.header.stamp=rospy.Time()
    #     point_.point.x= data.pose.position.x
    #     point_.point.y= data.pose.position.y
    #     point_.point.z= data.pose.position.z
    
    #     result =self.listener.transformPoint("base_link",point_)
    #     # print(result)
    #     # if(self.start_calibration == False):
    #     #     start_calibration_x = abs(self.arm.get_current_pose().pose.position.x - result.point.x) - 0.45
    #     #     start_calibration_y = self.arm.get_current_pose().pose.position.y - result.point.y
    #     #     start_calibration_z = self.arm.get_current_pose().pose.position.z - result.point.z
    #     #     self.start_calibration = self.follow_cartesian_path(start_calibration_x, start_calibration_y, start_calibration_z)
    #     #     if(self.start_calibration):
    #     #         print("初始化矫正成功")
    #     #     else:
    #     #         print("初始化矫正失败")
    #     #         self.start_calibration = True
    #     if(self.change_start_pose):
    #         self.start_pose.pose.position.x = result.point.x
    #         self.start_pose.pose.position.y = result.point.y
    #         self.start_pose.pose.position.z = result.point.z
    #         self.change_start_pose = False
    #     else:
    #         self.change_start_pose = self.conpute_qr_move(result)
    
    # def conpute_qr_move(self,qr_point):

    #     self.ar_x = qr_point.point.x - self.start_pose.pose.position.x
    #     self.ar_y = qr_point.point.y - self.start_pose.pose.position.y
    #     self.ar_z = qr_point.point.z - self.start_pose.pose.position.z

    #     if( abs(self.ar_x)>=0.01 or abs(self.ar_y)>=0.01 or abs(self.ar_z)>=0.01 ):
    #         print("开始跟随")
    #         self.count += 1 
    #         follow_res = self.follow_cartesian_path(self.ar_x, self.ar_y, self.ar_z)
    #         if(follow_res == True):
    #             result = "成功"
    #             # self.start_pose.pose.position.x = result_.point.x #+ self.ar_x
    #             # self.start_pose.pose.position.y = result_.point.y #+ self.ar_y
    #             # self.start_pose.pose.position.z = result_.point.z #+ self.ar_z
    #             if(self.fail_count):
    #                 self.fail_count = 0
    #                 self.Accumulate_x = 0
    #                 self.Accumulate_y = 0
    #                 self.Accumulate_z = 0
    #             print("第{0}次跟随{1}".format(self.count,result))
    #             rospy.sleep(0.5)
    #             return True
    #         else:
    #             result = "失败"
    #             self.fail_count += 1 
    #             if(self.Accumulate_x != self.ar_x or self.Accumulate_y != self.ar_y or self.Accumulate_z != self.ar_z):
    #                 self.Accumulate_x += self.ar_x
    #                 self.Accumulate_y += self.ar_y
    #                 self.Accumulate_z += self.ar_z
    #                 print("累加量")
    #                 print(self.Accumulate_x, self.Accumulate_y, self.Accumulate_z)
    #             print("第{0}次跟随{1}".format(self.count,result))
    #             # rospy.sleep(0.5)
    #             if(self.fail_count >=3):
    #                 return True
    #             return False
    #     else:
    #         print("catch Qr position")
    #         return False
    
    def follow_ar_pose_one(self,data):
        
        if(self.follow_res):
            self.ar_point.header.stamp = rospy.Time()
            self.ar_point.pose.position.x = data.pose.position.x
            self.ar_point.pose.position.y = data.pose.position.y
            self.ar_point.pose.position.z = data.pose.position.z

        ar_res_point = self.listener.transformPose("base_link",self.ar_point)
        # ar_res_orion = self.listener.transformOr
        arm_pose = self.arm.get_current_pose().pose
        print(ar_res_point)
        
        r = math.atan2(2 * (data.pose.orientation.w * data.pose.orientation.x + data.pose.orientation.y * data.pose.orientation.z), 
                    1 - 2 * (data.pose.orientation.x * data.pose.orientation.x + data.pose.orientation.y * data.pose.orientation.y))
        r = (r / math.pi) * 180

        p = math.asin(2 * (data.pose.orientation.w * data.pose.orientation.y - 
                            data.pose.orientation.z * data.pose.orientation.x))
        p = (p / math.pi) * 180

        y = math.atan2(2 * (data.pose.orientation.w * data.pose.orientation.z + data.pose.orientation.x * data.pose.orientation.y), 
                    1 - 2 * (data.pose.orientation.y * data.pose.orientation.y + data.pose.orientation.z * data.pose.orientation.z))
        y = (y / math.pi) * 180

        ar_arm_x = ar_res_point.pose.position.x - (arm_pose.position.x+0.25)#(arm_pose.position.x+0.35)
        ar_arm_y = ar_res_point.pose.position.y - arm_pose.position.y#(arm_pose.position.y+0.01)  
        ar_arm_z = ar_res_point.pose.position.z - arm_pose.position.z#(arm_pose.position.z-0.05)  

        print("绕x轴旋转角度: {}".format(r))
        print("绕y轴旋转角度: {}".format(p))
        print("绕z轴旋转角度: {}\n".format(y))

        # if (abs(ar_arm_x) >= 0.365):
        #     arm_move_x = ar_arm_x - 0.35

        if(abs(ar_arm_x)>0.20 or abs(ar_arm_y)>0.015 or abs(ar_arm_z>0.015)):
            print("开始跟随")
            self.count += 1
            self.follow_res = self.cartesian_move_path(data.pose.orientation, ar_arm_x, ar_arm_y, ar_arm_z)
            if(self.follow_res):
                res = "成功"
                print("第{0}次跟随{1}".format(self.count,res))
                rospy.sleep(0.5)
            else:
                res = "失败"
                print("第{0}次跟随{1}".format(self.count,res))
                self.fail_count += 1
                if(self.fail_count>3):
                    self.follow_res = True
                    self.fail_count = 0
                rospy.sleep(0.5)
        else:
            print("catch Qr position")


    def cartesian_move_path(self,ar_orientation,target_x,target_y,target_z):           

        #当前位置为第一个点
        self.waypoints.append(self.arm.get_current_pose().pose)
        wpose = Pose()
        wpose1 = Pose()
        #路径点成功率
        fraction = 0.0

        max_fraction = 0.0
        
        wpose.position = self.waypoints[0].position 
        wpose.orientation = self.waypoints[0].orientation
        wpose1.orientation = wpose.orientation

        #ar_orientation
        
        # print(wpose)

        #添加路径点
        print("位移量")
        print(target_x,target_y,target_z)
        for i in range(0,3):

            wpose1.position.x =  wpose.position.x + (target_x/3)*(i+1)
       
            wpose1.position.y =  wpose.position.y + (target_y/3)*(i+1)
       
            wpose1.position.z =  wpose.position.z + (target_z/3)*(i+1)

            # print(wpose1.position.x,wpose1.position.y,wpose1.position.z)
            self.waypoints.append(copy.deepcopy(wpose1))
        print(wpose1)
        # print(self.waypoints)
        #规划100次，全部点规划成功fraction=1跳出，执行
        for i in range(0,50):
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

        #大于80%执行
        if max_fraction >= 0.75:
            self.arm.execute(self.max_fraction_plan,wait = True)
            del self.waypoints[:]
            return True
        else:
            #print("Motion Msg param [barm] = FALSE!")
            del self.waypoints[:]
            return False


    def follow_cartesian_path(self,ar_orientation,move_x,move_y,move_z):           

        #当前位置为第一个点
        self.waypoints.append(self.arm.get_current_pose().pose)
        wpose = Pose()
        wpose1 = Pose()
        #路径点成功率
        fraction = 0.0

        max_fraction = 0.0
        
        wpose.position = self.waypoints[0].position 
        wpose.orientation = self.waypoints[0].orientation
        wpose1.orientation = wpose.orientation

        #ar_orientation
        
        # print(wpose)

        #添加路径点
        print("位移量")
        print(move_x,move_y,move_z)
        for i in range(0,3):

            wpose1.position.x =  wpose.position.x + (move_x/3)*(i+1)
       
            wpose1.position.y =  wpose.position.y + (move_y/3)*(i+1)
       
            wpose1.position.z =  wpose.position.z + (move_z/3)*(i+1)

            # print(wpose1.position.x,wpose1.position.y,wpose1.position.z)
            self.waypoints.append(copy.deepcopy(wpose1))
        print(wpose1)
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

        #大于80%执行
        if max_fraction >= 0.75:
            self.arm.execute(self.max_fraction_plan,wait = True)
            del self.waypoints[:]
            return True
        else:
            #print("Motion Msg param [barm] = FALSE!")
            del self.waypoints[:]
            return False
        

if __name__ == "__main__":
    test = Fllow_ArCode()
    test.start_fllow()
        
