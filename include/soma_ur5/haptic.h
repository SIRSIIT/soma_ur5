#ifndef INCLUDE_SOMA_UR5_HAPTIC_H_
#define INCLUDE_SOMA_UR5_HAPTIC_H_


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
//#include <soma_ur5/ch_mapping.h>
#include "soma_ur5/haptic_guidance.h"
#include "scoop_msgs/Scoop.h"

class Haptic{

public:
    //haptic_guidance gui;
    //Haptic(haptic_guidance());
    Haptic(haptic_guidance &);
    //Haptic();
    ~Haptic();
    ros::NodeHandle *nh;
    bool haptic_loop(haptic_guidance &);
    //bool haptic_loop();
    dynamic_reconfigure::Server<soma_ur5::dyn_ur5_hapticConfig> config_server;



private:
    ros::Subscriber sub_pose,sub_ft,sub_grip,sub_force, sub_force_bl, sub_mapping;
    ros::Publisher pub_hap_pose,pub_robot_com,pub_grip,pub_pedal, pub_d_pose, dbg_pub_grip, dbg_robot_com;
    ros::ServiceClient ft_client;
    //ros::ServiceServer service = nh->advertiseService("change_mapping_server", &Haptic::change_mapping,this);
    ros::Time last_command, aux; 
    geometry_msgs::PoseStamped ee_pose,hap_pose,hap_pose_initial,ee_pose_initial, camera_pose, camera_to_base;
    geometry_msgs::WrenchStamped cur_ee_force, cur_ee_force_bl, paletta_torque;
    //geometry_msgs::Point fake_gripper;
    scoop_msgs::Scoop fake_gripper;


    double scale_factor;
    bool pedal_on;
    double grip_val;
    std_msgs::Int32 map;
    int hap_gui;
    int mapping=3;
    double fx,fy,fz;
    int initialize_haptic();
    //bool autonomous_motion(int mapping, geometry_msgs::PoseStamped);
    
    //bool change_mapping(soma_ur5::ch_mapping::Request &request, soma_ur5::ch_mapping::Response &response);
    void mapping_callback(const std_msgs::Int32::ConstPtr& msg);
    bool GetHapticInfo(geometry_msgs::Pose &h_pose);
    bool SetHaptic(int &, haptic_guidance &, int &);
    //bool SetHaptic(int &);
    void robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void grip_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg); //std_msgs::Float64::ConstPtr
    void ee_force_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void ee_force_bl_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    bool move_haptic(geometry_msgs::Pose);
    bool goto_initial();
    geometry_msgs::Pose diff_pose(geometry_msgs::Pose, int &);
    geometry_msgs::Pose scale_pose(geometry_msgs::Pose in, std::string mode);
    void config_cb(soma_ur5::dyn_ur5_hapticConfig &config, uint32_t level);
    void bias_sensor();
};

#endif /* INCLUDE_SOMA_UR5_HAPTIC_H_ */
