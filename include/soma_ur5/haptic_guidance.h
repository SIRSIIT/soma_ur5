//
// Created by enrico on 10/09/2020.
//

#ifndef SOMA_UR5_HAPTIC_GUIDANCE_H
#define SOMA_UR5_HAPTIC_GUIDANCE_H

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <soma_ur5/dhdc.h>
#include <soma_ur5/drdc.h>
#include <soma_ur5/utils.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/WrenchStamped.h>
#include <dynamic_reconfigure/server.h>
#include <soma_ur5/dyn_ur5_hapticConfig.h>
#include <soma_ur5/dhdc.h>
#include <soma_ur5/drdc.h>
//#include "soma_ur5/meshgrid.hpp"
#include <vector>
#include <opencv2/core/core.hpp>
#include <math.h>
#include <cmath>
#include <iomanip>
#include <iostream>


class haptic_guidance {

public:
    haptic_guidance();
    //~haptic_guidance();
    //ros::NodeHandle *nh;
    void init();
    //bool guidance_loop();
    //std::tuple<double,double,double> guidance_loop();
    std::tuple<double,double,double> guidance_loop(geometry_msgs::PoseStamped);
    std::tuple<std::vector<cv::Mat>, std::vector<cv::Mat>, std::vector<cv::Mat>> meshgrid(const cv::Range &xgv, const cv::Range &ygv, const cv::Range &zgv);
    std::tuple<std::vector<cv::Mat>, std::vector<cv::Mat>, std::vector<cv::Mat>>  gradient(std::vector<cv::Mat> &);
    std::vector<cv::Mat> X, Y, Z, Gx, Gy, Gz;
    std::vector<int> xi,yi,zi;
    int goal[3];
    int range[6] = {-10, 100, 0, 100, 0, 100}; // Potential Field Size (in cm)
    std::vector<cv::Mat> potential_field( std::vector<cv::Mat> &, std::vector<cv::Mat> &, std::vector<cv::Mat> &, int []);
    std::vector<cv::Mat> diff_vector(const cv::Mat &, int);
    std::vector<cv::Mat> gradient_computation(std::vector<cv::Mat> &f, cv::Mat &h, std::vector<cv::Mat> &H, int &);
    bool SetHaptic(double &, double &, double &);
    //dynamic_reconfigure::Server<soma_ur5::dyn_ur5_hapticConfig> config_server;
private:
    ros::Subscriber sub_pose, sub_obj_pose, sub_hap_pedal;
    ros::Publisher pub_hap_pose,pub_robot_com,pub_grip,pub_pedal, pub_d_pose;
    ros::ServiceClient ft_client;
    //ros::ServiceServer service = nh->advertiseService("change_mapping_server", &Haptic::change_mapping,this);
    //ros::Time last_command, aux;
    geometry_msgs::PoseStamped ee_pose, cur_ee_pose, obj_pose;
    bool pedal_on=false;
    std::vector<int>::iterator x_itr, y_itr, z_itr, gx_itr, gy_itr, gz_itr;
    int x_idx, y_idx, z_idx, gx_idx, gy_idx, gz_idx;
    double fx, fy, fz;
    void robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void hap_pedal_cb(const std_msgs::Bool::ConstPtr &msg);
    //void object_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    //void ee_force_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    //void ee_force_bl_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);

    //void config_cb(soma_ur5::dyn_ur5_hapticConfig &config, uint32_t level);
    //void bias_sensor();

};

#endif //SOMA_UR5_HAPTIC_GUIDANCE_H