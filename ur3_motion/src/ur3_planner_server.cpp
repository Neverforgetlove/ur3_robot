#include "ur3_motion/ur3_planner_server.h"

using namespace std;

motion_plan_server::motion_plan_server(ros::NodeHandle &n):
    node(n),
    plan_server(node,"ur3_motion_plan",
                boost::bind(&motion_plan_server::goalCB, this, _1),
                boost::bind(&motion_plan_server::cancelCB, this, _1),
                false),
    control_client("ur3_controller",true),
    motion_mag(node),
    has_active_goal(false)
    {
        execute_timer  = node.createTimer(ros::Duration(0.0333333), &motion_plan_server::listenState, this);
        watchdog_timer = node.createTimer(ros::Duration(1.0), &motion_plan_server::watchdog, this);
        plan_server.start();
    };

motion_plan_server::~motion_plan_server(){
    watchdog_timer.stop();
    execute_timer.stop();
    ros::shutdown();
};

void motion_plan_server::superStop() {
    has_active_goal = true;
    control_client.cancelAllGoals();
    active_goal.setCanceled();
    while(!goal_queue.empty())
        goal_queue.pop();
    while(!cancel_queue.empty())
        cancel_queue.pop();
    has_active_goal = false;
};

//定时监听服务器状态   根据控制器频率定时触发
void motion_plan_server::listenState(const ros::TimerEvent &e){
    if(!has_active_goal && !goal_queue.empty()){
        has_active_goal = true;
        std::thread exec_thread(std::bind(&motion_plan_server::serverExecute, this));
        exec_thread.detach();
    }
};

void motion_plan_server::judgeAbort(){
    if(has_active_goal){
        if(ros::Time::now() > start_time +ros::Duration(30.0)){
            //时间超限就进行抢占
            ROS_ERROR("Aborted!! task_id : [%s] -- Motion plan Time out!",active_goal.getGoal()->header.task_id.c_str());
            superStop();
            return ;
        }
    }
};

void motion_plan_server::watchdog(const ros::TimerEvent &e){
    judgeAbort();
};

void motion_plan_server::serverExecute(){
    if(!goal_queue.empty()){
        GoalHandle gh = goal_queue.top();
        goal_queue.pop();
        if(!cancel_queue.empty() && (gh == cancel_queue.top())){
            cancel_queue.pop();
        }else{
            active_goal = gh;
            start_time = ros::Time::now();
            ROS_INFO("Current Active Control Goal -- task_id : [%s], task_weight : [%d], goal_seq : [%d]",
                gh.getGoal()->header.task_id.c_str(), (int)gh.getGoal()->header.task_weight, (int)gh.getGoal()->header.seq);
            ur3_msgs::MotionPlanGoal goal;
            goal.header = gh.getGoal()->header;
            goal.msp = gh.getGoal()->msp;

            if(!motion_mag.Update(goal)){
                ROS_ERROR("Task_di : [%s] Motion_plan Error!", goal.header.task_id.c_str());
                has_active_goal = false;
                superStop();
                return ;
            }else{
                callController(motion_mag.control_goal);
            }
        }
    }
    ros::Duration(2.0).sleep();
    has_active_goal = false;
};

void motion_plan_server::goalCB(GoalHandle gh){
    goal_queue.push(gh);
    ROS_INFO("Goal pushed into the queue!");
    gh.setAccepted();
};

void motion_plan_server::cancelCB(GoalHandle gh){
    if(active_goal == gh){
        active_goal.setCanceled();
        has_active_goal = false;
    }else{
        cancel_queue.push(gh); //保存 关闭队列 等待比对
    }
};

void motion_plan_server::callController(ur3_msgs::BaseControlGoal &goal){
    control_client.sendGoal(goal);
    ros::Duration(0.03333).sleep();
};