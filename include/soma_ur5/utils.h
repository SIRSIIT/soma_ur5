#ifndef INCLUDE_SOMA_UR5_UTILS_H_
#define INCLUDE_SOMA_UR5_UTILS_H_



#include <Eigen/SVD>
#include <Eigen/Dense>
#include <math.h>
#include <tf2/utils.h>
#include <ros/ros.h>

typedef Eigen::Matrix< double, 6, 6 > Matrix6d;
typedef Eigen::Matrix< double, 6, 1 > Vector6d;

#define xyz_eq(a,b) {b.x=a.x;b.y=a.y;b.z=a.z;}

namespace utils{
void array2pose(double pos[16],geometry_msgs::Pose &pose_out,tf2::Transform &tra){

    tra.setBasis(tf2::Matrix3x3(
                     pos[0],pos[1],pos[2],
            pos[4],pos[5],pos[6],
            pos[8],pos[9],pos[10]));
    tra.setOrigin(tf2::Vector3(pos[3],pos[7],pos[11]));

    geometry_msgs::Quaternion q_pose;

    pose_out.position.x=pos[3];
    pose_out.position.y=pos[7];
    pose_out.position.z=pos[11];

 //   tf::quaternionTFToMsg(tra.getRotation(),q_pose);
  tf2::convert(tra.getRotation(),q_pose);
    pose_out.orientation=q_pose;


}
double constrainAngle(double x){
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
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

std::string print_matrix(int m,int n, double* M, std::string prefix) {
    std::ostringstream dbg_msg;
    dbg_msg.setf(std::ios_base::fixed);
    dbg_msg << prefix << std::setprecision(4) << "\n";
    for(int i=0;i<m;i++){
        for(int j=0;j<n;j++){
            dbg_msg <<  M[i*m+j] <<  "\t";
        }
        dbg_msg <<  "\n";
    }
    dbg_msg <<  "\n";
    return dbg_msg.str();
}

void pose2array(geometry_msgs::Pose msg,double T_pose[16]){
    tf2::Quaternion tf_quat;

   // tf::quaternionMsgToTF(msg.orientation,tf_quat);
    tf2::convert(msg.orientation,tf_quat);
    tf2::Transform Tg;
    Tg.setRotation(tf_quat);
    Tg.setOrigin(tf2::Vector3(msg.position.x,msg.position.y,msg.position.z));

    // Tg=Tb_ee; //Try giving the current pose to inverse kinematics
    tf2::Vector3 el,dv;
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

geometry_msgs::Pose Transform2Pose(tf2::Transform in){
    geometry_msgs::Pose out;

    out.orientation.x=in.getRotation().x();
    out.orientation.y=in.getRotation().y();
    out.orientation.z=in.getRotation().z();
    out.orientation.w=in.getRotation().w();
    out.position.x=in.getOrigin().x();
    out.position.y=in.getOrigin().y();
    out.position.z=in.getOrigin().z();

    return out;

}
tf2::Transform Pose2Transform(geometry_msgs::Pose in){
tf2::Transform out;

tf2::Quaternion q;
q.setX(in.orientation.x);
q.setY(in.orientation.y);
q.setZ(in.orientation.z);
q.setW(in.orientation.w);
out.setRotation(q);
out.setOrigin(tf2::Vector3(in.position.x,in.position.y,in.position.z));

return out;
}
}
#endif /* INCLUDE_SOMA_UR5_UTILS_H_ */
