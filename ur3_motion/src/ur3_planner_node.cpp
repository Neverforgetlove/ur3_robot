#include <ros/ros.h>
#include <ur3_motion/ur3_planner_server.h>
using namespace std;

int main(int argc, char** argv){
	ros::init(argc, argv, "ur3_planner_node");
    ros::NodeHandle node;
    
    //实例化对象
    motion_plan_server mps(node);

    ros::spin();
    
    return 0;
}