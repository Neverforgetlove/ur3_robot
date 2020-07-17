cmake_minimum_required(VERSION 3.0.2)
project(ur3_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  moveit_msgs
  control_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
)

add_message_files(DIRECTORY msg
FILES
  TaskStep.msg
  MotionStep.msg
  Header.msg
  ControlStep.msg
)

add_service_files(DIRECTORY srv
)

add_action_files(DIRECTORY action
FILES
  PandaTask.action
  MotionPlan.action
  BaseControl.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
  trajectory_msgs
  moveit_msgs
  control_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime
  moveit_msgs 
  actionlib_msgs 
  trajectory_msgs 
  geometry_msgs 
  control_msgs 
  std_msgs
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
# # include
# # ${catkin_INCLUDE_DIRS}
# )
