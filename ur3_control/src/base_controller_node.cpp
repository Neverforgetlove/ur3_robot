#include <ros/ros.h>
#include <ur3_control/base_controller_server.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "ur3_control_node");
    ros::NodeHandle node("~");

    std::string arm, gripper;
    if(!(node.getParam("ur3_arm_command_topic",arm))){
       ROS_ERROR("No ur3_arm_command_topic parameters provided");
       return 1; 
    }
    if(!(node.getParam("ur3_gripper_command_topic",gripper))){
       ROS_ERROR("No ur3_gripper_command_topic parameters provided");
       return 1; 
    }
    ur3_controller_server pcs(node, arm, gripper);
    ros::spin();
    
    return 0;
}