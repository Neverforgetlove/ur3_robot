/*
运动规划单元
*/

#include <ros/ros.h>
#include <cmath>
#include <cstring>
#include <vector>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>

#include <ur3_msgs/MotionStep.h>
#include <ur3_msgs/BaseControlGoal.h>
#include <ur3_msgs/MotionPlanGoal.h>
#include <ur3_motion/basic_kin.h>
#include <ur3_motion/traj_manager.h>

using namespace std;

class MotionManager
{
public:
    typedef bool(MotionManager::*motionPtr)(ur3_msgs::MotionStep);
    MotionManager(ros::NodeHandle &n);
     ~MotionManager();
     bool Init();
     bool Update(ur3_msgs::MotionPlanGoal &goal);
     ur3_msgs::BaseControlGoal control_goal;  //最终发送目标

private:

//规划结果轨迹整合到control_goal中

    //给control_goal 添加control_step
    bool AddControlStep(ur3_msgs::MotionStep &msg, moveit_msgs::RobotTrajectory trajectory);
    
    //时间参数化
    bool AddTimeParam(moveit_msgs::RobotTrajectory &trajectory, double v, double a);
    
    //静止状态 非静止则抢占  
    //用于后续任务规划等待或衔接
    bool Stop(ur3_msgs::MotionStep msg);

    //关节角按期望路径移动
    bool JointTrajCommand(ur3_msgs::MotionStep msg);

    //末端转换到期望姿态
    bool RotateToTargetPose(ur3_msgs::MotionStep msg);
    
    //末端移动到目标位置
    bool MoveToTargetPose(ur3_msgs::MotionStep msg);
    
    //跟随
    bool Follow(ur3_msgs::MotionStep msg);

    //末端笛卡尔移动或特定角度移动
    bool MoveByCartAngle(ur3_msgs::MotionStep msg);

    //特定关节控制
    bool JointAngleCommand(ur3_msgs::MotionStep msg);

    //当前运动规划的动作序列
    vector<ur3_msgs::MotionStep> current_msp;

    //函数指针
    vector<motionPtr> type_motions_ptr;

    traj_manager traj_mag;

    //基本移动的速度  加速的
    double vel, acc;

    //跟随的速度  加速度
    double follow_vel, follow_acc;

    //加载Moveit Robot相关对象
    ros::NodeHandle node;
    robot_model_loader::RobotModelLoader *robot_model_loader;
    robot_model::RobotModelPtr robot_model;
    planning_scene::PlanningScenePtr planning_scene;
    planning_pipeline::PlanningPipeline *planning_pipeline;
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    moveit_msgs::MotionPlanResponse response;

    //路径点
    std::vector<geometry_msgs::Pose> waypoints;

    //robot当前状态
    robot_state::RobotStatePtr current_state;
    const robot_model::JointModelGroup *joint_group_ptr;
    moveit::planning_interface::MoveGroupInterface *group_ptr;

    //执行时间
    double exec_duration;

    //当前关节值
    std::vector<double> joint_position;
    
    //末端当前姿态
    geometry_msgs::Pose current_pose;

};
