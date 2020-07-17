/*
约束motion plan路径
*/
#ifndef TRAJ_MANAGER_H
#define TRAJ_MANAGER_H

#include <ur3_motion/basic_kin.h>
#define crl_cyle 0.01

using namespace std;
class traj_manager
{
public:
    traj_manager();
    ~traj_manager();
    traj_manager(Eigen::MatrixXd q);
    traj_manager(std::vector<double> q);
    traj_manager(Eigen::MatrixXd q, Eigen::Affine3d Twd2target);
    traj_manager(std::vector<double> q, Eigen::Affine3d Twd2target);

    basic_kin bk;

    Eigen::MatrixXd q0;//初始的关节角
    Eigen::MatrixXd qtmp;//关节角的中间变量
    Eigen::Affine3d Twd2f;
    Eigen::Affine3d Twd2f_target;
    Eigen::Affine3d Twd2f_tmp;

    void setStartPoint(Eigen::MatrixXd q);
    void setStartPoint(std::vector<double> q);
    void setTargetPoint(Eigen::Affine3d Twd2target);

    vector<Eigen::MatrixXd> q_traj;
    vector<Eigen::MatrixXd> GenTraj_GP(double tf = 1.0);//利用梯度投影法生成轨迹

    std::vector<geometry_msgs::Pose> waypoints; //航点
    trajectory_msgs::JointTrajectory trajectory; //arm的运动轨迹

};

#endif