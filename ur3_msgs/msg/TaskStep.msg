#ur3_msgs/Header header

int8 task_type                      #任务分步类型 -- 待定
bool track_bgrasp                   #夹爪命令
string str_id                       #试校ID号
geometry_msgs/Pose target_pose      #末端目标在世界坐标系中的位姿
string object_name                  #物体名称
string joint_id                     #关节号
float64[] joint_goal                #单个关节角或夹爪闭合
float64[] task_duration             #姿态跟随运行时长

