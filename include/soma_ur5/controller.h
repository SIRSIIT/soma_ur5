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
#include <stdlib.h>
//#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <vector>
#include <array>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/server.h>
#include <soma_ur5/dyn_ur5_controllerConfig.h>
#include <ctime>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <soma_ur5/utils.h>
#include <boost/algorithm/string.hpp>
//#define M_PI 3.14159265
#define MAX_SPEED 0.3


typedef Eigen::Matrix< double, 6, 6 > Matrix6d;
typedef Eigen::Matrix< double, 6, 1 > Vector6d;

class UR5_Control {

public:
	UR5_Control();
	ros::NodeHandle *nh;    
    enum solver_t{CLOSED_FORM,JACOBIAN};
    dynamic_reconfigure::Server<soma_ur5::dyn_ur5_controllerConfig> config_server;

protected:
    std::array<int,6> jo;
	trajectory_msgs::JointTrajectory trajectory;
	control_msgs::FollowJointTrajectoryGoal goal_traj;
	sensor_msgs::JointState cur_joints;
    ros::Subscriber sub_joints,sub_goal_pose;
    ros::Publisher pub_ee_pose,speed_command;
    tf2_ros::TransformBroadcaster tf_br;
    tf2_ros::TransformListener *tf_list;
    tf2_ros::Buffer buffer;
    bool init,using_gazebo;
    tf2::Transform Tb_ee;
    double speed_gain;

    std::map<std::string,double> map_j_lim,map_ws_lim;
    std::string control_topic;
    double max_angle;
    int solver;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *act_client;



    bool valid_jconf(double joints[6]);
    void config_cb(soma_ur5::dyn_ur5_controllerConfig &config, uint32_t level);
	void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void goal_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    bool send_joint_command(double[6]);
    bool send_speed_command(double[6]);
    bool choose_sol(int nsols, double* sols, double* best, double &max_cost);
    bool closed_form(double* goal,double* comm);
    bool jac_based(double* goal,double* comm);
    void calculate_jac(double cur_q[], Matrix6d &J);
    Vector6d fwd_kin(double q[6]);

};

#endif /* INCLUDE_SOMA_UR5_CONTROLLER_H_ */
