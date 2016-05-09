/*
    * controller.h
    *
    *  Created on: May 7, 2016
    *      Author: joao bimbo
    */


#ifndef INCLUDE_SOMA_UR5_MOTION_KDL_H_
#define INCLUDE_SOMA_UR5_MOTION_KDL_H_
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <soma_ur5/ur5_motion.h>

class UR5_Motion_KDL : UR5_Motion{
public:
    UR5_Motion_KDL(std::string description) : UR5_Motion(){
        kdl_parser::treeFromParam(description,robot_tree);
        robot_tree.getChain("base_link","ee_link",robot_chain);
        joints.resize(robot_chain.getNrOfJoints());
        fwd_kin_solver=new KDL::ChainFkSolverPos_recursive(robot_chain);
    }

    void updateJoints(sensor_msgs::JointState jnts){
        cur_joints=jnts;
        for (int i=0;i<cur_joints.name.size();i++){
            joints.q(i)=cur_joints.position.at(i);
            joints.qdot(i)=cur_joints.velocity.at(i);
        }

    }
    trajectory_msgs::JointTrajectory calcSpeeds(geometry_msgs::Pose cur_pose, geometry_msgs::Pose goal_pose, double gain){

    }

    geometry_msgs::Pose getEEpose(){
        KDL::Frame p_kdl;
        geometry_msgs::Pose  p_out;
        fwd_kin_solver->JntToCart(joints.q,p_kdl);
        tf::poseKDLToMsg(p_kdl,p_out);
        return p_out;
    }

void getJacobian(){

    KDL::ChainIkSolverVel_pinv_nso *inv_kin_solver=new KDL::ChainIkSolverVel_pinv_nso(robot_chain);

    KDL::Frame p_kdl;
    fwd_kin_solver->JntToCart(joints.q,p_kdl);

    KDL::VectorVel vv(KDL::Vector(0,0,0.1),KDL::Vector(0.2,0.3,0.1));
    KDL::Twist tw=KDL::Twist(KDL::Vector(0,0,0.1),KDL::Vector(0.2,0.3,0.1));
    KDL::FrameVel fv=KDL::FrameVel(p_kdl,tw);
    KDL::JntArrayVel jv=KDL::JntArrayVel(6);

    ROS_INFO("v: %f %f %f %f %f %f",fv.p.p(0),fv.p.p(1),fv.p.p(2),fv.p.v(0),fv.p.v(1),fv.p.v(2));

    inv_kin_solver->CartToJnt(joints.q,fv,jv);

    ROS_INFO("FV: %f %f %f %f %f %f",jv.qdot(0),jv.qdot(1),jv.qdot(2),jv.qdot(3),jv.qdot(4),jv.qdot(5));
    //inv_kin_solver->CartToJnt(joints.q,tw,jv);
    ROS_INFO("TW: %f %f %f %f %f %f",jv.qdot(0),jv.qdot(1),jv.qdot(2),jv.qdot(3),jv.qdot(4),jv.qdot(5));
}


protected:
    KDL::Tree robot_tree;
    KDL::Chain robot_chain;
    KDL::ChainFkSolverPos_recursive *fwd_kin_solver;
    KDL::JntArrayAcc joints;




};


#endif /* INCLUDE_SOMA_UR5_MOTION_KDL_H_ */


