#include <ur3_motion/motion_manager.h>
using namespace std;

MotionManager::MotionManager(ros::NodeHandle &n):
        node(n),
        traj_mag()
{
    //函数指针
    type_motions_ptr.resize(7);
    type_motions_ptr[0] = &MotionManager::Stop;
    type_motions_ptr[1] = &MotionManager::JointTrajCommand;
    type_motions_ptr[2] = &MotionManager::RotateToTargetPose;
    type_motions_ptr[3] = &MotionManager::MoveToTargetPose;
    type_motions_ptr[4] = &MotionManager::Follow;
    type_motions_ptr[5] = &MotionManager::MoveByCartAngle;
    type_motions_ptr[6] = &MotionManager::JointAngleCommand;

    ros::AsyncSpinner spinner(3);
    spinner.start();
    
    traj_mag.setStartPoint(joint_position);

    //Move_Group
    static const std::string Planning_Group = "ur3_arm";
    group_ptr = new moveit::planning_interface::MoveGroupInterface(Planning_Group);
    current_state = group_ptr->getCurrentState();
    joint_group_ptr = current_state->getJointModelGroup(Planning_Group);
    current_state->copyJointGroupPositions(joint_group_ptr, joint_position);
    current_pose = group_ptr->getCurrentPose().pose;
    for(int i =0;i<6;i++){
        ROS_INFO("joint_angle --- %f",joint_position[i]);
    }

    robot_model_loader = new robot_model_loader::RobotModelLoader("robot_description");
    robot_model = robot_model_loader->getModel();
    planning_scene = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(robot_model));
    planning_scene->setCurrentState(*current_state);
    planning_pipeline = new planning_pipeline::PlanningPipeline(robot_model, node, "/move_group/planning_plugin", "/move_group/request_adapters");
    req.group_name = Planning_Group;
};

MotionManager::~MotionManager(){}


bool MotionManager::Init(){

    //获取yaml配置
    if(!(node.getParam("motion_config/vel",vel))){
        ROS_ERROR("No motion_config/vel parameters parameters provided");
        return false;
    }

    if(!(node.getParam("motion_config/acc",acc))){
        ROS_ERROR("No motion_config/acc parameters parameters provided");
        return false;
    }

    if(!(node.getParam("motion_config/follow_vel",follow_vel))){
        ROS_ERROR("No motion_config/follow_vel parameters parameters provided");
        return false;
    }

    if(!(node.getParam("motion_config/follow_acc",follow_acc))){
        ROS_ERROR("No motion_config/follow_acc parameters parameters provided");
        return false;
    }
    waypoints.clear();
    ur3_msgs::BaseControlGoal tmp_goal;
    control_goal = tmp_goal;
    control_goal.exec_duration = 0;
    return true;
};

bool MotionManager::Update(ur3_msgs::MotionPlanGoal &goal){
    if(!Init()){
        ROS_ERROR("Motion Manager Init FALSE");
        return false;
    }
    control_goal.header = goal.header;
    current_msp = goal.msp;
    ROS_INFO("Motion_Planning task_id:[%s] step_size[%d]", goal.header.task_id.c_str(), (int)current_msp.size());
    for(int i =0;i  < current_msp.size(); i++){
        int move_type = current_msp[i].move_type;
        if(!(this->*(type_motions_ptr[move_type]))(current_msp[i])){
            ROS_ERROR("MOTION_PLANNING_ERROR task_id: [%s] FAILED  msp index: [%d]",goal.header.task_id.c_str(), i);
            return false;            
        }
    }
    return true;
};

bool MotionManager::AddControlStep(ur3_msgs::MotionStep &msg, moveit_msgs::RobotTrajectory trajectory){
    ur3_msgs::ControlStep tmp_step;
    control_goal.exec_duration += 1.0;
    if(msg.barm){
        tmp_step.barm = true;
        tmp_step.joint_trajectory = trajectory;
        if(trajectory.joint_trajectory.points.size()){
            control_goal.exec_duration += trajectory.joint_trajectory.points.back().time_from_start.toSec();
            joint_position = trajectory.joint_trajectory.points.back().positions;
            current_state->setJointGroupPositions(joint_group_ptr, joint_position);
        }
    }else{
        tmp_step.barm = false;
    }
    if(msg.bgrasp){
        tmp_step.bgrasp = true;
        tmp_step.on_off = msg.on_off;
    }else{
        tmp_step.bgrasp = false;
    }
    control_goal.csp.push_back(tmp_step);

    //更新状态
    planning_scene->setCurrentState(*current_state);
    group_ptr->setStartState(*current_state);
    traj_mag.setStartPoint(joint_position);
    return true;
};

bool MotionManager::AddTimeParam(moveit_msgs::RobotTrajectory &trajectory, double v, double a){
    robot_trajectory::RobotTrajectory rt(robot_model,"ur3_arm");
    rt.setRobotTrajectoryMsg(*current_state, trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool ItSuccess = iptp.computeTimeStamps(rt, v, a);
    rt.getRobotTrajectoryMsg(trajectory);
    return true;
};

bool MotionManager::Stop(ur3_msgs::MotionStep msg){
    moveit_msgs::RobotTrajectory empty;
    empty.joint_trajectory.joint_names.resize(6);
    empty.joint_trajectory.joint_names = joint_group_ptr->getVariableNames();
    AddControlStep(msg, empty);
    return true;
};

bool MotionManager::JointTrajCommand(ur3_msgs::MotionStep msg){
    if(msg.barm){

        //初始化
        tf::poseEigenToMsg(traj_mag.bk._Twd2f, current_pose);
        msg.F_pose.orientation = current_pose.orientation;
        Eigen::Affine3d F_pose;
        tf::poseMsgToEigen(msg.F_pose, F_pose);
        traj_mag.setStartPoint(joint_position);
        traj_mag.setTargetPoint(F_pose);

        //临时轨迹，用于时间参数化
        moveit_msgs::RobotTrajectory tmp_traj;
        tmp_traj.joint_trajectory.joint_names = joint_group_ptr->getVariableNames();
        
        //计算路径点
        vector<Eigen::MatrixXd> traj = traj_mag.GenTraj_GP(1);
        int points = traj.size();
        tmp_traj.joint_trajectory.points.resize(points);
        for(int i = 0; i < points; i++){
            for(int j = 0; j < 6; j++){
                tmp_traj.joint_trajectory.points[i].positions.push_back(traj[i](j,0));
            }
        }
        AddTimeParam(tmp_traj, vel, acc);
        AddControlStep(msg, tmp_traj);
    }else{
        ROS_ERROR("Motion Msg param [baram] = False");
        return false;
    }
    return true;
};

bool MotionManager::RotateToTargetPose(ur3_msgs::MotionStep msg){
    if(msg.barm){
        joint_position[5] = 0.5;
        robot_state::RobotState goal_state(robot_model);
        goal_state.setJointGroupPositions(joint_group_ptr,joint_position);
        moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_group_ptr);
        req.goal_constraints.clear(); 
        req.goal_constraints.push_back(joint_goal);
        planning_pipeline->generatePlan(planning_scene, req, res);
        if(res.error_code_.val != res.error_code_.SUCCESS){
            ROS_ERROR("Could not compute plan successfully");
            return false;
        }
        res.getMessage(response);
        AddTimeParam(response.trajectory, vel , acc);
        //赋值
        AddControlStep(msg,response.trajectory); 
    }else{
        ROS_ERROR("Motion Msg param [barm] = FALSE!");
        return false;
    }
    return true;
};

bool MotionManager::MoveToTargetPose(ur3_msgs::MotionStep msg){
	if(msg.barm){
		if(msg.joint_id.size() != msg.joint_goal.size()){
			ROS_ERROR("ControlJoint Size  !=  Joint_command Size");
			return false;
		}
		geometry_msgs::PoseStamped pose;
		std::vector<double> tolerance_pose(3, 0.01);
		std::vector<double> tolerance_angle(3, 0.01);
		pose.header.frame_id = "base_link";
		pose.pose = msg.F_pose;
		pose.pose.orientation.w = msg.F_pose.orientation.w;
		moveit_msgs::Constraints pose_goal = 
						kinematic_constraints::constructGoalConstraints("tool0", pose, tolerance_pose, tolerance_angle);
		req.goal_constraints.clear();
		req.goal_constraints.push_back(pose_goal);
		planning_pipeline->generatePlan(planning_scene, req, res);
		if (res.error_code_.val != res.error_code_.SUCCESS)
		{
  			ROS_ERROR("Could not compute plan successfully");
  			return 0;
		}
		AddControlStep(msg,response.trajectory);
	}else{
		ROS_ERROR("Motion Msg param [barm] = FALSE!");
		return false;
	}
    return true;
};

bool MotionManager::Follow(ur3_msgs::MotionStep msg){
    return true;
};

bool MotionManager::MoveByCartAngle(ur3_msgs::MotionStep msg){
	moveit_msgs::RobotTrajectory tmp_traj;
	if(msg.barm){
		tf::poseEigenToMsg(traj_mag.bk._Twd2w, current_pose);
		geometry_msgs::Pose target_pose = current_pose;
		target_pose.position = msg.F_pose.position;
		waypoints.push_back(target_pose);
		double fraction = 0.0;
		int compute_num = 0;
		for(int count = 0; fraction < 1.0; count++){
			 fraction = group_ptr->computeCartesianPath(waypoints,0.001,0.0,tmp_traj);
			 if(fraction == 1.0){
				 break;
			 }
			 if(count >= 100){
				ROS_ERROR("Can't Compute CartesianPath!");
				return false;
			 }
		 }
		waypoints.clear();		
		AddTimeParam(tmp_traj, vel, acc);
	}
	AddControlStep(msg,tmp_traj);
	return true;
};

bool MotionManager::JointAngleCommand(ur3_msgs::MotionStep msg){
	if(msg.barm){
		if(msg.joint_id.size() != msg.joint_goal.size()){
			ROS_ERROR("ControlJoint Size  !=  Joint_command Size");
			return false;
		}
		for(int i = 0; i < msg.joint_id.size(); i++){
			joint_position[msg.joint_id[i] - '0'] = msg.joint_goal[i];
		}
		robot_state::RobotState goal_state(robot_model);
		goal_state.setJointGroupPositions(joint_group_ptr, joint_position);
      	moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_group_ptr);
      	req.goal_constraints.clear();
      	req.goal_constraints.push_back(joint_goal);
		planning_pipeline->generatePlan(planning_scene, req, res);
		
		if (res.error_code_.val != res.error_code_.SUCCESS) {
			ROS_ERROR("Could not compute plan successfully");
			return false;
		}
		res.getMessage(response);
	}
	AddTimeParam(response.trajectory,vel, acc);
	AddControlStep(msg,response.trajectory);
    return true;    
};