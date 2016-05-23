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
#include <qb_interface/handRef.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>

class GazeboBridge {

public:
	GazeboBridge();
	ros::NodeHandle *nh;    
    void run();

protected:
    void vel_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
    void pos_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
    void joint_callback(const sensor_msgs::JointState::ConstPtr &msg);    
    void hand_callback(const qb_interface::handRef::ConstPtr &msg);
    void sensor_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    bool bias_srv_cb(std_srvs::Empty::Request &req,std_srvs::Empty::Response &rsp);
    void keep_position();

    ros::Subscriber sub_comm,sub_joints,sub_traj,sub_hand,sub_sensor;
    std::vector<ros::Publisher> pub_vels;
    ros::Publisher pub_hand,pub_sensor;
    ros::Time last;
    sensor_msgs::JointState cur_joints,jp_to_keep;
    bool position_mode, velocity_mode_before, started,using_gazebo;
    std::vector<std::string> joint_names{"shoulder_pan_joint","shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    std::vector<int> jo;
    geometry_msgs::Wrench sensor_offset;
    geometry_msgs::WrenchStamped last_sensor_reading;
    ros::ServiceServer bias_srv;

};

#endif /* INCLUDE_SOMA_UR5_GAZEBO_BRIDGE_H_ */
