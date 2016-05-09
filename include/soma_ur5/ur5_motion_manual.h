/*
    * controller.h
    *
    *  Created on: May 7, 2016
    *      Author: joao bimbo
    */


#ifndef INCLUDE_SOMA_UR5_MOTION_MANUAL_H_
#define INCLUDE_SOMA_UR5_MOTION_MANUAL_H_
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <soma_ur5/ur5_motion.h>
#include <ur_kinematics/ur_kin.h>
#include <Eigen/Dense>
#include <soma_ur5/utils.h>



typedef Eigen::Matrix< double, 6, 6 > Matrix6d;
typedef Eigen::Matrix< double, 6, 1 > Vector6d;

class UR5_Motion_Manual : UR5_Motion{
public:
    UR5_Motion_Manual() : UR5_Motion(){


    }

    void updateJoints(sensor_msgs::JointState jnts){
        cur_joints=jnts;
    }

    geometry_msgs::Pose getEEpose(){
        geometry_msgs::Pose poseEE;
        double cur_q[6],aT_ee[16];
        for(int i=0;i<6;i++) {
            cur_q[i]=cur_joints.position.at(i);
        }
        ur_kinematics::forward(cur_q,aT_ee);
        utils::array2pose(aT_ee,poseEE,Tb_ee);
        return poseEE;
    }

    trajectory_msgs::JointTrajectory calcSpeeds(geometry_msgs::Pose cur_pose, geometry_msgs::Pose goal_pose,double gain){
        trajectory_msgs::JointTrajectory traj;
        Vector6d deltaX,deltaTh;
        Matrix6d Jacobian;

        deltaX=getDeltaX(cur_pose,goal_pose);
        Jacobian=getJacobian(cur_joints);
        deltaTh=utils::pseudoinv(Jacobian)*deltaX;

        trajectory_msgs::JointTrajectoryPoint p;
        for(int i=0;i<cur_joints.name.size();i++){
            traj.joint_names.push_back(cur_joints.name.at(i));
        }
        for(int i=0;i<cur_joints.name.size();i++){
            p.velocities.push_back(deltaTh[i]*gain);
        }

        traj.header.stamp=ros::Time::now();
        traj.points.push_back(p);
        return traj;
    }


protected:
    Vector6d fwd_kin(double q[6]){
        double r,p,y;
        Vector6d fw;
        double T_j[16];
        ur_kinematics::forward(q,T_j);
        tf2::Matrix3x3(T_j[0],T_j[1],T_j[2],
                T_j[4],T_j[5],T_j[6],
                T_j[8],T_j[9],T_j[10]).getRPY(r,p,y);
        fw << T_j[3], T_j[7], T_j[11], r, p, y;
        return fw;

    }

    Vector6d getDeltaX(geometry_msgs::Pose cur,geometry_msgs::Pose goal){
        tf2::Transform T_cur=Tb_ee;
        tf2::Transform T_goal;
        double rc,pc,yc,rg,pg,yg;
        Vector6d delta_x;

        T_goal=utils::Pose2Transform(goal);
        T_cur.getBasis().getRPY(rc,pc,yc);
        T_goal.getBasis().getRPY(rg,pg,yg);
        delta_x  << T_goal.getOrigin().getX()-T_cur.getOrigin().getX(),
                T_goal.getOrigin().getY()-T_cur.getOrigin().getY(),
                T_goal.getOrigin().getZ()-T_cur.getOrigin().getZ(),
                utils::constrainAngle(rg-rc),utils::constrainAngle(pg-pc),utils::constrainAngle(yg-yc);
        return delta_x;
    }

    Matrix6d getJacobian(sensor_msgs::JointState j){
        Matrix6d J;
        Vector6d cur_c,nc;
        double cur_q[j.name.size()],next_q[j.name.size()];
        double h=0.001;

        for(int i=0;i<6;i++) cur_q[i]=j.position.at(i);
        cur_c=fwd_kin(cur_q);

        for(int i=0;i<6;i++) {
            for(int j=0;j<6;j++) {
                next_q[j]=cur_q[j];
            }
            next_q[i]+=h;
            nc=fwd_kin(next_q);
            J.col(i) = 1/h*(nc-cur_c).col(0);
        }
        return J;
    }

};



#endif /* INCLUDE_SOMA_UR5_MOTION_KDL_MANUAL_H_ */
