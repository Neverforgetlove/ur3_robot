/*
ur3控制Action服务端，底层通信单元
*/

#ifndef UR3_CONTROLLER_SERVER
#define UR3_CONTROLLER_SERVER

#include <iostream>
#include <queue>
#include <vector>
#include <functional>
#include <algorithm>
#include <thread>
#include <boost/thread/thread.hpp>

#include <ros/service.h>
#include <ros/time.h>
#include <ros/duration.h>

/*#include<franka_gripper/franka_gripper.h>*/
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>


#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/GripperCommandAction.h>
#include <ur3_msgs/BaseControlGoal.h>
#include <ur3_msgs/ControlStep.h>
#include <ur3_msgs/BaseControlAction.h>

using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperCommandClient;
typedef actionlib::ActionServer<ur3_msgs::BaseControlAction> BaseControlServer;
typedef BaseControlServer::GoalHandle GoalHandle;

struct goal_cmp {
    bool operator () (const GoalHandle &a, const GoalHandle &b){
        //权重相同时，序列号最小值优先
        if(a.getGoal()->header.task_weight == b.getGoal()->header.task_weight){
            return a.getGoal()->header.seq > b.getGoal()->header.seq;
        }
        return a.getGoal()->header.task_weight < b.getGoal()->header.task_weight;
    } 
};

class ur3_controller_server{
    public:

        ur3_controller_server(ros::NodeHandle &n, string arm_topic, string gripper_topic);
        ~ur3_controller_server();
        void Stop();
        
    private:

        void listenState(const ros::TimerEvent &e); //监听目标队列
        void serverExecute();                        //多线程运行  
        void judgeAbort();                           //目标抢占
        void watchdog(const ros::TimerEvent &e);      //定时运行，定时监听错误
        void goalCB(GoalHandle gh);
        void cancelCB(GoalHandle gh);

        //夹爪部分
        
        void ActiveCb();
        void DoneCb(const actionlib::SimpleClientGoalState& state,
                    const control_msgs::GripperCommandResultConstPtr& result);
        void FeedbackCb(const control_msgs::GripperCommandFeedbackConstPtr& feedback);
        

        //服务端请求处理
        bool GripperCommand(bool on_off);
        void JointCommand(trajectory_msgs::JointTrajectory trajectory);
        ros::NodeHandle node;
        ros::Publisher pub_joint_command;
        ros::Timer watchdog_timer;
        ros::Timer execute_timer;

        std::priority_queue<GoalHandle, std::vector<GoalHandle>, goal_cmp>goal_queue;    //保存目标队列
        std::priority_queue<GoalHandle, std::vector<GoalHandle>, goal_cmp>cancel_queue;  //空间换时间  O(1)删除

        GoalHandle active_goal;
        GripperCommandClient client;
        BaseControlServer server;
        std::vector<std::string> joint_names;
        bool has_active_goal;
        bool is_grasped;

        double start_time;     //开始时间
        double duration_time;  //等待时间

};

#endif