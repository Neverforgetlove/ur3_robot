#ur3_msgs/Header header    #头消息

int8 move_type                                          #移动类型  -待确定
bool barm                                               #是否控制手臂
bool bgrasp                                             #是否控制夹爪
bool on_off                                             #夹爪控制开关 
geometry_msgs/Pose F_pose                               #末端姿态
string joint_id                                         #控制的关节号
float64[] joint_goal                                    #控制的关节号对应的关节角
trajectory_msgs/JointTrajectoryPoint[] waypoints        #期望路径中间点
float64 follow_duration                                 #姿态跟踪的运行时长限制