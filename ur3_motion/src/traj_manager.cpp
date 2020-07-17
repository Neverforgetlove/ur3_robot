#include <ur3_motion/traj_manager.h>

traj_manager::traj_manager()
{

}

traj_manager::~traj_manager()
{

}

traj_manager::traj_manager(Eigen::MatrixXd q)
{
    setStartPoint(q);
}

traj_manager::traj_manager(std::vector<double> q)
{
    setStartPoint(q);
}

traj_manager::traj_manager(Eigen::MatrixXd q, Eigen::Affine3d Twd2target)
{
    setStartPoint(q);
    setTargetPoint(Twd2target);
}

traj_manager::traj_manager(std::vector<double> q, Eigen::Affine3d Twd2target)
{
    cout<<"初始化开始"<<endl;

    setStartPoint(q);
    setTargetPoint(Twd2target);
    cout<<q0.transpose()<<endl;
    cout<<"初始化完成"<<endl;
}

void traj_manager::setStartPoint(Eigen::MatrixXd q)
{
    q0.resize(6,1);
    qtmp.resize(6,1);

    q0 = q;
    qtmp = q;
    bk.update(q0);
    Twd2f = bk._Twd2f;
}

void traj_manager::setStartPoint(std::vector<double> q)
{
    q0.resize(6,1);
    qtmp.resize(6,1);

    for(int i=0;i<6;i++){
    q0(i,0) = q[i];
    qtmp(i,0) = q[i];
    }    
    bk.update(q0);
    Twd2f = bk._Twd2f;
}

void traj_manager::setTargetPoint(Eigen::Affine3d Twd2target)
{
    Twd2f_target = Twd2target;
}

vector<Eigen::MatrixXd> traj_manager::GenTraj_GP(double tf)
{
    int n_q = (int)(tf/crl_cyle);
    double k = 1/(double(n_q));

    bk.update(qtmp);
    bk.CalT2new_f(Twd2f_target);
    Eigen::MatrixXd dx_target = bk.dX;

    qtmp = q0;
    q_traj.resize(n_q);
    for(int i=0;i<n_q;i++){
        q_traj[i].resize(7,1);

        k = 1/(double(n_q - i));
        bk.update(qtmp);
        bk.CalT2new_f(Twd2f_target);
        std::vector<double> qc = bk.Calqc(k);

        for(int j=0;j<6;j++){
            q_traj[i](j,0) = qc[j];
            qtmp(j,0) = qc[j];
        }
    }
    return q_traj;
};