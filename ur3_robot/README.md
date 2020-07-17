#   安装依赖
```bash
sudo apt-get install ros-$ROS_DISTRO-moveit-* \
                     ros-$ROS_DISTRO-manipulation-msgs \
                     ros-$ROS_DISTRO-gazebo-ros-pkgs \
                     ros-$ROS_DISTRO-gazebo-ros-control \
                     ros-$ROS_DISTRO-industrial-core \
                     ros-$ROS_DISTRO-trac-ik-kinematics-plugin \
                     ros-kinetic-industrial-msgs
```

#   仿真环境测试
```bash
roslaunch ur3_camera_moveit_config demo.launch
```

#   启动pc与机械臂通信
```bash
#   启动ur_modern_driver
roslaunch ur_modern_driver ur3_bringup_joint_limited.launch robot_ip:=192.168.1.102
#   启动moveit_planning
roslaunch ur3_camera_moveit_config ur3_camera_moveit_planning_execution.launch
#   在rviz导入模型
roslaunch ur3_camera_moveit_config moveit_rviz.launch config:=true
```

#   控制机械臂
## 获取机械臂当前位置
### 关节角度
```bash
#   查询关节角度，position为六个关节角度
rostopic echo /joint_states
```
在ur_planning/scripts/joint_planning.py文件下，把相应的六个关节角度替换掉，就可以控制机器人到达相应的位姿
```bash
rosrun ur_planning joint_planning.py
```

### 位姿
```bash
#   获取当前机械臂的位姿，Translation为位置，Quaternion为四元数
rosrun tf tf_echo base_link ee_link
```
在/ur_planning/scripts/pose_planning.py文件下，把相应的位置和四元数替换，就可以控制机器人到达相应的位姿
```bash
rosrun ur_planning pose_planning.py
```


