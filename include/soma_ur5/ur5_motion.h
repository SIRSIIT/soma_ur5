/*
    * controller.h
    *
    *  Created on: May 7, 2016
    *      Author: joao bimbo
    */


#ifndef INCLUDE_SOMA_UR5_MOTION_H_
#define INCLUDE_SOMA_UR5_MOTION_H_
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
class UR5_Motion{
public:
    UR5_Motion(){

    }
    ~UR5_Motion(){ }
    virtual void updateJoints(sensor_msgs::JointState jnts) = 0;
    virtual geometry_msgs::Pose getEEpose() = 0;
    virtual trajectory_msgs::JointTrajectory calcSpeeds(geometry_msgs::Pose cur_pose,geometry_msgs::Pose goal_pose,double gain) = 0;



protected:
    sensor_msgs::JointState cur_joints;
    tf2::Transform Tb_ee;

};



#endif /* INCLUDE_SOMA_UR5_MOTION_KDL_H_ */
