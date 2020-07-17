#include <ur3_motion/basic_kin.h>
#include <cmath>

using namespace std;

basic_kin::basic_kin()
{
    _Tw2f = Eigen::Affine3d::Identity();
    _Tw2f.rotate(Eigen::AngleAxisd(-0.785398163397, Eigen::Vector3d::UnitZ()).toRotationMatrix());
    _Tw2f.pretranslate(Eigen::Vector3d(0.0, 0.0, 0.1));
    _Tf2w = Eigen::Affine3d::Identity();
    _Tf2w.matrix() = _Tw2f.matrix().inverse();

    robot_model_loader::RobotModelLoader robot_model_loader_tmp("robot_description");
    robot_model_loader = new robot_model_loader::RobotModelLoader("robot_description");
    robot_model = robot_model_loader->getModel();

    robot_state_ptr = new robot_state::RobotState(robot_model);
    joint_group_ptr = robot_model->getJointModelGroup("ur3_arm");
    const std::vector<std::string>& joint_names = joint_group_ptr->getVariableNames();
    robot_state_ptr->copyJointGroupPositions(joint_group_ptr, joint_values);

    joint_command.resize(6);
    joint_low_limits = {-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-2.8973};
    joint_up_limits = {2.8973,1.7628,2.8973,-0.0698,2.8973,2.8973};
    dX.resize(5,1);
    dq.resize(6,1);

    kstep = 0.01;
}

basic_kin::~basic_kin()
{
    delete robot_state_ptr;
    delete robot_model_loader;
}

void basic_kin::updatejoints(std::vector<double> q)
{
    for(int i=0;i<6;i++){
        joint_values[i] = q[i];
    }
    robot_state_ptr->setJointGroupPositions(joint_group_ptr, joint_up_limits);

}

void basic_kin::updatejoints(std::vector<float> q)
{
    for(int i =0;i<6;i++){
        joint_values[i] = (double)q[i];
    }
    robot_state_ptr->setJointGroupPositions(joint_group_ptr, joint_up_limits);
}

void  basic_kin::updatejoints(Eigen::VectorXd q)
{
    for(int i=0;i<7;i++){
        joint_values[i] = q(i);
    }
    robot_state_ptr->setJointGroupPositions(joint_group_ptr, joint_values);
}

void basic_kin::updatejoints(Eigen::MatrixXd q)
{
    for(int i =0;i<6;i++){
        joint_values[i] = q(i,0);
    }
    robot_state_ptr->setJointGroupPositions(joint_group_ptr, joint_up_limits);
}

void basic_kin::update(std::vector<double> q)
{
    updatejoints(q); 
    kinematic(); 
}

void basic_kin::update(std::vector<float> q)
{
    updatejoints(q); 
    kinematic(); 
}

void basic_kin::update(Eigen::VectorXd q)
{
    updatejoints(q); 
    kinematic(); 
}

void basic_kin::update(Eigen::MatrixXd q)
{
    updatejoints(q); 
    kinematic(); 
}

void basic_kin::kinematic(){
    const Eigen::Affine3d& wrist_state =  robot_state_ptr->getGlobalLinkTransform("tool0");
    _Twd2w = wrist_state;
    _Twd2f = _Twd2w*_Tw2f;

    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    robot_state_ptr->getJacobian(joint_group_ptr,
                               robot_state_ptr->getLinkModel(joint_group_ptr->getLinkModelNames().back()),
                               reference_point_position, _Jac);
    _Jacinv = pinv(_Jac);   
}

void basic_kin::CalT2new_f(Eigen::Affine3d target_finger_state)
{
    Eigen::Affine3d target_wrist_state;
    target_wrist_state.matrix() = target_finger_state.matrix() * _Tf2w.matrix();
    basic_kin::CalT2new_w(target_wrist_state);
    CaldX();
}

void basic_kin::CalT2new_w(Eigen::Affine3d target_wrist_state)
{
    // cout<<"bk中的手腕姿态 \n"<<_Twd2w.matrix()<<endl;
    // cout<<"bk中的目标姿态 \n"<<target_wrist_state.matrix()<<endl;
    Tw2wnew.matrix() =   _Twd2w.matrix().inverse() * target_wrist_state.matrix();
    Tf2fnew.matrix() =  _Tf2w.matrix() * Tw2wnew.matrix() *_Tw2f.matrix();
    CaldX();
}

void basic_kin::CaldX()
{
    dxyz_wrist_local = Tw2wnew.translation();
    drpy_wrist_local = rotationMatrixToEulerAngles(Tw2wnew.rotation());

    dxyz_wrist_global = _Twd2w.rotation()*dxyz_wrist_local;
    drpy_wrist_global = _Twd2w.rotation()*drpy_wrist_local; 
    dX<<dxyz_wrist_global(0,0),dxyz_wrist_global(1,0),dxyz_wrist_global(2,0),
    drpy_wrist_global(0,0),drpy_wrist_global(1,0),drpy_wrist_global(2,0);   
}

std::vector<double> basic_kin::Calqc(double k)
{
    // cout<<dX.transpose()<<endl;
    dq = k*_Jacinv*dX;
    for (int i=0;i<6;i++){
        joint_command[i] = joint_values[i] + dq(i,0); 
        if(joint_command[i]>joint_up_limits[i]-0.02)
        {
          joint_command[i]=joint_up_limits[i]-0.02;  
        }
        if(joint_command[i]<joint_low_limits[i]+0.02)
        {
          joint_command[i]=joint_low_limits[i]+0.02;  
        }
    }

    return joint_command;
}

Eigen::Affine3d basic_kin::CalGraspState(Eigen::Affine3d obj_state)
{
    Eigen::Affine3d grasp_state = Eigen::Affine3d::Identity();

    Eigen::Matrix4d object_state = obj_state.matrix();      //obj in world frame
    double pitch=0.0;
    double yaw = 0.0;
    // switch (obj_tpye)
    // {
        // case BALL:
            yaw = atan2(object_state(1,3),object_state(0,3));
            // cout<<"step 1 obj in ur3 frame:\n"<<object_state<<endl;

            //desired position in ur3 frame
            grasp_state.pretranslate(Eigen::Vector3d(object_state(0,3),object_state(1,3),object_state(2,3)));
            // cout<<"step 2 grasp postion:\n"<<grasp_state.matrix()<<endl;

            //desired attitude in ur3 frame
            grasp_state.rotate(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix());
            // cout<<"step 3 grasp yaw:\n"<<grasp_state.matrix()<<endl;
            grasp_state.rotate(Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()).toRotationMatrix());
            // cout<<"step 4 grasp pitch:\n"<<grasp_state.matrix()<<endl;
            
            //desired finger in ur3 frame
            grasp_state.matrix() = grasp_state.matrix() * _Twd2f.matrix();
            // cout<<"step 5 finger:\n"<<grasp_state.matrix()<<endl;

            //desired wrist in ur3 frame
            grasp_state.matrix() = grasp_state.matrix() *_Tw2f.matrix().inverse();
            // cout<<"step 6 wrist: \n"<<grasp_state.matrix()<<endl;
            return grasp_state;
    //         /* code for condition */
    //         break;
    //     case CYLINDER:

    //         break;
    //     case BOX:

    //         break;
    //     default:
    //         yaw = atan2(object_state(1,3),object_state(0,3));
    //         grasp_state.pretranslate(obj_state.translation());
    //         grasp_state.rotate(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix());
    //         grasp_state.rotate(Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()).toRotationMatrix());

    //         return grasp_state;
    //         break;
    //         /* Default Code */
    // }
    
}





bool basic_kin::isRotationMatrix(Eigen::Matrix3d  R) 
{ 
    Eigen::Matrix3d Rt = R.transpose(); 
    Eigen::Matrix3d shouldBeIdentity = Rt * R - Eigen::Matrix3d::Identity(); 
    
    return shouldBeIdentity.norm() < 1e-6; 
}


Eigen::Vector3d basic_kin::rotationMatrixToEulerAngles(Eigen::Matrix3d R) 
{
    // assert(isRotationMatrix(R)); 
    float sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0) ); 
    bool singular = sy < 1e-6; 
    // If 
    float x, y, z; 
    if (!singular) 
    { 
        x = atan2(R(2,1) , R(2,2)); 
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0)); }
    else 
    { 
     x = atan2(-R(1,2), R(1,1));
      y = atan2(-R(2,0), sy); 
      z = 0; 
      }
    return Eigen::Vector3d(x, y, z); 
 }

Eigen::MatrixXd basic_kin::pinv(Eigen::MatrixXd  B)
{
    Eigen::MatrixXd A = B.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);//M=USV*
    double  pinvtoler = 1.e-8; //tolerance
    int row = A.rows();
    int col = A.cols();
    int k = min(row,col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col,row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i<k; ++i) {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i) 
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X=(svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());//X=VS+U*
    return X.transpose();
}