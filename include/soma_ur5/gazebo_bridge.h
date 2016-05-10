/*
 * gazebo_bridge.h
 *
 *  Created on: Mar 7, 2016
 *      Author: joao
 */

#ifndef INCLUDE_SOMA_UR5_GAZEBO_BRIDGE_H_
#define INCLUDE_SOMA_UR5_GAZEBO_BRIDGE_H_

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <ctime>
#include <boost/algorithm/string.hpp>

class GazeboBridge {

public:
	GazeboBridge();
	ros::NodeHandle *nh;    
    void run();

protected:
    void vel_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
    void pos_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
    void joint_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void keep_position();
    ros::Subscriber sub_comm,sub_joints,sub_traj;
    std::vector<ros::Publisher> pub_vels;
    ros::Time last;
    sensor_msgs::JointState cur_joints,jp_to_keep;
    bool position_mode, velocity_mode_before, started,using_gazebo;
    std::vector<std::string> joint_names{"shoulder_pan_joint","shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    std::vector<int> jo;

};

#endif /* INCLUDE_SOMA_UR5_GAZEBO_BRIDGE_H_ */
