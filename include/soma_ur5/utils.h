#include <soma_ur5/controller.h>
typedef Eigen::Matrix< double, 6, 6 > Matrix6d;
typedef Eigen::Matrix< double, 6, 1 > Vector6d;

namespace utils{
void array2pose(double pos[16],geometry_msgs::Pose &pose_out,tf::Transform &tra){

    tra.setBasis(tf::Matrix3x3(
                       pos[0],pos[1],pos[2],
            pos[4],pos[5],pos[6],
            pos[8],pos[9],pos[10]));
    tra.setOrigin(tf::Vector3(pos[3],pos[7],pos[11]));

    geometry_msgs::Quaternion q_pose;

    pose_out.position.x=pos[3];
    pose_out.position.y=pos[7];
    pose_out.position.z=pos[11];

    tf::quaternionTFToMsg(tra.getRotation(),q_pose);
    pose_out.orientation=q_pose;


}


Matrix6d pseudoinv(Matrix6d m){
    Matrix6d pinv_m;
    Matrix6d m2=m;
    Vector6d S;
    Matrix6d I;

    double tolerance;
    double epsilon=std::numeric_limits<double>::epsilon();

    I.setIdentity();


    Eigen::JacobiSVD<Eigen::MatrixXd> svd(m2,Eigen::ComputeThinU | Eigen::ComputeThinV);

    tolerance=epsilon*std::max(m.cols(),m.rows())*svd.singularValues().array().abs().maxCoeff();
    tolerance=1e-40;

    S=svd.singularValues().array();
    for(int i=0;i<svd.singularValues().size();i++){
        if(S(i)<tolerance) S(i)=0;
        else S(i)=1/S(i);
    }


    I=S.asDiagonal();
    pinv_m=svd.matrixV() * I * svd.matrixU().adjoint();
    return pinv_m;
}



void pose2array(geometry_msgs::Pose msg,double T_pose[16]){
    tf::Quaternion tf_quat;

    tf::quaternionMsgToTF(msg.orientation,tf_quat);
    tf::Transform Tg;
    Tg.setRotation(tf_quat);
    Tg.setOrigin(tf::Vector3(msg.position.x,msg.position.y,msg.position.z));

    // Tg=Tb_ee; //Try giving the current pose to inverse kinematics
    tf::Vector3 el,dv;
    dv=Tg.getOrigin();
    el=Tg.getBasis().getRow(0);
    T_pose[0]=el.getX();
    T_pose[1]=el.getY();
    T_pose[2]=el.getZ();
    T_pose[3]=dv.getX();
    el=Tg.getBasis().getRow(1);
    T_pose[4]=el.getX();
    T_pose[5]=el.getY();
    T_pose[6]=el.getZ();
    T_pose[7]=dv.getY();
    el=Tg.getBasis().getRow(2);
    T_pose[8]=el.getX();
    T_pose[9]=el.getY();
    T_pose[10]=el.getZ();
    T_pose[11]=dv.getZ();
    T_pose[12]=0;
    T_pose[13]=0;
    T_pose[14]=0;
    T_pose[15]=1;
}
}
