/*
*service客户端 与ur3_util通信
*Action服务端  与ur3_planner_node通信
*对运动规划请求生成轨迹或处理轨迹 多轨迹衔接等
*结合视觉检测的环境进行规划和ur3_task_manager结果
*结合moveit中的 motion planner adapter进行运动规划
*/


#ifndef UR3_PLANNER_SERVER_H
#define UR3_PLANNER_SERVER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <queue>
#include <algorithm>
#include <thread>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>

#include <ur3_motion/motion_manager.h>
#include <ur3_msgs/BaseControlAction.h>
#include <ur3_msgs/MotionPlanAction.h>

using namespace std;

#define PI 3.1415926535897932346

typedef actionlib::ActionServer<ur3_msgs::MotionPlanAction> MotionPlanServer;
typedef actionlib::SimpleActionClient<ur3_msgs::BaseControlAction> BaseControlClient;
typedef MotionPlanServer::GoalHandle GoalHandle;

struct goal_cmp {
    bool operator () (const GoalHandle &a, const GoalHandle &b){
        //权重相同时，序列号最小值优先
        if(a.getGoal()->header.task_weight == b.getGoal()->header.task_weight){
            return a.getGoal()->header.seq > b.getGoal()->header.seq;
        }
        return a.getGoal()->header.task_weight < b.getGoal()->header.task_weight;
    } 
};

class motion_plan_server
{
public:
    motion_plan_server(ros::NodeHandle &n);
    ~motion_plan_server();
    void superStop(); //用于抢占/停止程序
private:
    //通信和运动规划
    void listenState(const ros::TimerEvent &e); //监听目标队列程序
    void serverExecute(); //多线程运行函数
    void judgeAbort();//判断是否抢占目标
    void watchdog(const ros::TimerEvent &e);  //看门狗函数 服务端运行过程中监听错误
    
    //Action 服务端相关
    void goalCB(GoalHandle gh); 
    void cancelCB(GoalHandle gh);
    
    //BaseControl 客户端
    void callController(ur3_msgs::BaseControlGoal &goal);
    ros::NodeHandle node;
    MotionPlanServer plan_server;
    BaseControlClient control_client;
    ros::Timer watchdog_timer;
    ros::Timer execute_timer;

    MotionManager motion_mag;

    std::priority_queue<GoalHandle,std::vector<GoalHandle>, goal_cmp>goal_queue;//目标的队列，进行保存
    std::priority_queue<GoalHandle,std::vector<GoalHandle>, goal_cmp>cancel_queue;//空间换时间，O(1)删除

    bool has_active_goal;//判断是否有目标在规划
    int current_task_weight; //当前运行的任务的权重
    int waiting_task_weight; //等待规划运行的任务权重
    GoalHandle active_goal; //目前规划的目标
    ros::Time start_time; //开始时间
    std::vector<std::string> joint_names; 
};


#endif