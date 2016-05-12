/*
 * controller.h
 *
 *  Created on: May 7, 2016
 *      Author: joao bimbo
 */


#ifndef INCLUDE_SOMA_UR5_CONTROLLER_MAIN_H_
#define INCLUDE_SOMA_UR5_CONTROLLER_MAIN_H_


#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <soma_ur5/dyn_ur5_controllerConfig.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <array>
#include <vector>
#include <map>
#include <boost/algorithm/string.hpp>
//#include <soma_ur5/ur5_motion_kdl.h>
#include <soma_ur5/ur5_motion_manual.h>

class UR5_Control_ROS{
public:
    ros::NodeHandle *nh;
    enum solver_t{CLOSED_FORM,JACOBIAN};
    dynamic_reconfigure::Server<soma_ur5::dyn_ur5_controllerConfig> config_server;
    UR5_Motion_Manual *ur5_model;

    UR5_Control_ROS(){
        this->nh=new ros::NodeHandle();

        ur5_model=new UR5_Motion_Manual();
        tf_list=new tf2_ros::TransformListener(buffer);
        sub_joints = nh->subscribe("joint_states", 1000, &UR5_Control_ROS::joint_state_callback, this);
        init=false;    using_gazebo=false;
        cur_joints.name.resize(6);cur_joints.position.resize(6);
        cur_joints.velocity.resize(6);cur_joints.effort.resize(6);
        pub_ee_pose = nh->advertise<geometry_msgs::PoseStamped>("ee_pose",5);

        dynamic_reconfigure::Server<soma_ur5::dyn_ur5_controllerConfig>::CallbackType f;
        f=boost::bind(&UR5_Control_ROS::config_cb, this, _1, _2);
        config_server.setCallback(f);
        while(!init){
            ros::spinOnce();
            ros::Rate(10).sleep();
        }

        speed_command = nh->advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed",5);
        ROS_INFO("Loading parameters...");
        nh->getParam("limits/workspace", map_ws_lim);
        nh->getParam("limits/joints", map_j_lim);
        nh->getParam("control_topic", control_topic);
        nh->getParam("limits/max_angle", max_angle);
        nh->getParam("limits/max_speed", max_speed);

        std::string solv_str;
        nh->getParam("solver",solv_str);

        if(boost::iequals(solv_str,"jacobian")) solver=UR5_Control_ROS::JACOBIAN;
        else if(boost::iequals(solv_str,"closed_form")) solver=UR5_Control_ROS::CLOSED_FORM;

        sub_goal_pose= nh->subscribe("goal_pose", 1000, &UR5_Control_ROS::goal_pose_callback, this);

    }
    void run();
    void joint_update(const sensor_msgs::JointState &jnt);
    trajectory_msgs::JointTrajectory  safety_enforcer(trajectory_msgs::JointTrajectory in);

protected:
    ros::Subscriber sub_joints,sub_goal_pose;
    ros::Publisher pub_ee_pose,speed_command;


    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg){
        if(!init){
            jo.resize(joint_names.size());
            for (int i=0;i<jo.size();i++){
                for (int j=0;j<msg->name.size();j++){
                    if(msg->name.at(j)==joint_names.at(i)){
                        jo[i]=j;
                    }
                }
            }
            if(boost::iequals(msg->name.at(0),"elbow_joint")) using_gazebo=true;
            else using_gazebo=false;
            init=true;
        }
        cur_joints.header=msg->header;
        for(int i=0;i<6;i++){
            cur_joints.name.at(i)=msg->name.at(jo[i]);
            cur_joints.position.at(i)=msg->position.at(jo[i]);
            cur_joints.velocity.at(i)=msg->velocity.at(jo[i]);
            cur_joints.effort.at(i)=msg->effort.at(jo[i]);
        }
        joint_update(cur_joints);

    }

    //'elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'


    void goal_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void config_cb(soma_ur5::dyn_ur5_controllerConfig &config, uint32_t level){
        ROS_DEBUG("Reconfigure Request.");
        speed_gain=config.speed_gain;

    }

    tf2_ros::TransformBroadcaster tf_br;
    tf2_ros::TransformListener *tf_list;
    double speed_gain;
    bool init,using_gazebo;
    tf2_ros::Buffer buffer;
    sensor_msgs::JointState cur_joints;
    std::vector<int> jo;
    std::map<std::string,double> map_j_lim,map_ws_lim;
    std::string control_topic;
    double max_angle,max_speed;
    int solver;
    std::vector<std::string> joint_names{"shoulder_pan_joint","shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

};
#endif /* INCLUDE_SOMA_UR5_CONTROLLER_MAIN_H_ */
