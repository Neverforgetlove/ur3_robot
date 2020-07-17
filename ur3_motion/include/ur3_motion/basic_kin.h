/*
运动学求解的相关运算
*/

#ifndef BASIC_KIN_H
#define BASIC_KIN_H

#include <iostream>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <cmath>
#include <vector>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

using namespace std;

const double PI = 3.1415926535897932346;

class basic_kin
{
public:
    enum OBJ_TYPE
    {
        BALL = 0, CYLINDER = 1, BOX = 2
    };
    Eigen::Affine3d _Tw2f;
    Eigen::Affine3d _Tf2w;
    basic_kin();
    ~basic_kin();

    Eigen::Affine3d CalGraspState(Eigen::Affine3d obj_state);

    //加载urdf模型，并构建Moveit对象，用于计算正逆运动学
    robot_model_loader::RobotModelLoader *robot_model_loader;
    robot_model::RobotModelPtr robot_model;
    robot_model::RobotState *robot_state_ptr;
    const robot_model::JointModelGroup *joint_group_ptr;
    std::vector<double> joint_values;
    std::vector<double> joint_low_limits;
    std::vector<double> joint_up_limits;
    double kstep;

    Eigen::Affine3d _Twd2f;  //末端夹具在世界坐标系中的位置
    Eigen::Affine3d _Twd2w;  //腕关节在世界坐标系中的位置
    Eigen::MatrixXd _Jac;    //雅克比矩阵
    Eigen::MatrixXd _Jacinv; //逆雅克比矩阵

    Eigen::Affine3d Tf2fnew;
    Eigen::Affine3d Tw2wnew;
    Eigen::Vector3d dxyz_wrist_local; // wrist的dxyz变换在wrist当地坐标系
    Eigen::Vector3d drpy_wrist_local; // wrist的drpy变换在wrist当地坐标系

    Eigen::Vector3d dxyz_wrist_global; // wrist的dxyz在世界坐标系
    Eigen::Vector3d drpy_wrist_global; // wrist的drpy在世界坐标系
    Eigen::MatrixXd dX;
    Eigen::MatrixXd dq;
    //执行命令
    std::vector<double> joint_command;
    // GripperControl gripper;
    trajectory_msgs::JointTrajectory trajectory; // arm的运动轨迹

    //正运动学
    void kinematic();

    void update(std::vector<double> q);
    void update(std::vector<float> q);
    void update(Eigen::VectorXd q);
    void update(Eigen::MatrixXd q);

    void CalT2new_f(
        Eigen::Affine3d
            target_finger_state); //根据finger期望位置计算finger到期望位置的变换
    void CalT2new_w(
        Eigen::Affine3d
            target_wrist_state); //根据wrist期望位置计算finger到期望位置的变换
    void CaldX();
    std::vector<double> Calqc(double k);

    // private:
    void updatejoints(std::vector<double> q);
    void updatejoints(std::vector<float> q);
    void updatejoints(Eigen::VectorXd q);
    void updatejoints(Eigen::MatrixXd q);

    // void updateGripper(std::double width);

    bool isRotationMatrix(Eigen::Matrix3d R);
    Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d R);
    Eigen::MatrixXd pinv(Eigen::MatrixXd B);
};

#endif