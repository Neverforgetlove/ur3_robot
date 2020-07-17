#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/duration.h>
#include <ur3_msgs/MotionStep.h>
#include <ur3_msgs/MotionPlanAction.h>
using namespace std;

typedef actionlib::SimpleActionClient<ur3_msgs::MotionPlanAction> Client;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "task_test_client");
    ros::NodeHandle node;
    Client client("ur3_motion_plan",true);
    ur3_msgs::MotionPlanGoal goal;
    goal.msp.resize(3);
    geometry_msgs::Pose pose;
    pose.position.x = 0.52027;
    pose.position.y = 0.07337;
    pose.position.z = 0.26111;
    pose.orientation.w = 0.0144;
    //pose.orientation.x = 1.0;
   // pose.orientation.y = 0.0;
   // pose.orientation.z = 0.0;
    vector<double> q0 = {0, -0.785, 0, -3*0.785, 0, 1.57, 0.785};
    int count = 1;
    while(ros::ok()){
        if(count % 2 == 0){
            q0 = {0, -0.785, 0, -3*0.785, 0, 1.57, 0.785};
        }else{
            q0 = {0, -0.785, 0, -3*0.785, 0, 0.57, 0.785};
        }
        goal.header.seq = 1;
        goal.header.task_id = '0' + count;
        goal.header.stamp = ros::Time::now();
        goal.header.task_weight = 3;

        goal.msp[0].barm = true;
        goal.msp[0].bgrasp = false;
        goal.msp[0].move_type = 6;
        goal.msp[0].joint_id = "0123456";
        goal.msp[0].joint_goal.resize(7);
        goal.msp[0].joint_goal = q0;

        goal.msp[1].barm = true;
        goal.msp[1].bgrasp = true;
        goal.msp[1].on_off = false;
        goal.msp[1].move_type = 5;
        goal.msp[1].F_pose = pose; 

        goal.msp[2].barm = true;
        goal.msp[2].bgrasp = true;
        goal.msp[2].on_off = true;
        goal.msp[2].move_type = 2;
        goal.msp[2].F_pose = pose; 

        client.sendGoal(goal);
        ROS_INFO("Send Task ID: [%d]", count++);
        ros::Duration(20.0).sleep();
    }
    return 0;
}