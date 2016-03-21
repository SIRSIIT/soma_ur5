/*
 * controller.h
 *
 *  Created on: Mar 7, 2016
 *      Author: joao
 */

#ifndef INCLUDE_SOMA_UR5_CONTROLLER_H_
#define INCLUDE_SOMA_UR5_CONTROLLER_H_

//#define UR10_PARAMS true
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <ur_kinematics/ur_kin.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <vector>
#include <array>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <ctime>
//#define M_PI 3.14159265

class UR5_Control {

public:
	UR5_Control();
	ros::NodeHandle *nh;    

protected:
    std::array<int,6> jo;
	trajectory_msgs::JointTrajectory trajectory;
	control_msgs::FollowJointTrajectoryGoal goal_traj;
	sensor_msgs::JointState cur_joints;
    ros::Subscriber sub_joints,sub_goal_pose;
    ros::Publisher pub_ee_pose;
    tf::TransformBroadcaster tf_br;
    tf::TransformListener tf_list;

    tf::Transform Tb_ee;

    std::map<std::string,double> map_j_lim,map_ws_lim;
    std::string control_topic;
    double max_angle;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *act_client;

    bool valid_jconf(double joints[6]);
	void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void goal_pose_callback(const geometry_msgs::Pose::ConstPtr &msg);
    bool send_joint_command(double[6]);
    std::string print_matrix(int m, int n, double* M, std::string prefix);
    bool choose_sol(int nsols, double* sols, double* best, double &max_cost);



};

#endif /* INCLUDE_SOMA_UR5_CONTROLLER_H_ */
