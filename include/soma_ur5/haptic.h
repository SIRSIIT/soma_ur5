#ifndef INCLUDE_SOMA_UR5_HAPTIC_H_
#define INCLUDE_SOMA_UR5_HAPTIC_H_


#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <soma_ur5/dhdc.h>
#include <soma_ur5/drdc.h>
#include <soma_ur5/utils.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/WrenchStamped.h>
#include <dynamic_reconfigure/server.h>
#include <soma_ur5/dyn_ur5_hapticConfig.h>


class Haptic{

public:
    Haptic();
    ~Haptic();
    ros::NodeHandle *nh;
    bool haptic_loop();
    dynamic_reconfigure::Server<soma_ur5::dyn_ur5_hapticConfig> config_server;

private:
    ros::Subscriber sub_pose,sub_ft,sub_grip,sub_force;
    ros::Publisher pub_hap_pose,pub_robot_com,pub_grip,pub_pedal;
    geometry_msgs::PoseStamped ee_pose,hap_pose,hap_pose_initial,ee_pose_initial;
    geometry_msgs::WrenchStamped cur_ee_force;

    double scale_factor;
    bool pedal_on;
    double grip_val;

    int initialize_haptic();
    bool GetHapticInfo(geometry_msgs::Pose &h_pose);
    bool SetHaptic();
    void robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void grip_callback(const std_msgs::Float64::ConstPtr &msg);
    void ee_force_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    bool move_haptic(geometry_msgs::Pose);
    bool goto_initial();
    geometry_msgs::Pose diff_pose(geometry_msgs::Pose);
    geometry_msgs::Pose scale_pose(geometry_msgs::Pose in, std::string mode);
    void config_cb(soma_ur5::dyn_ur5_hapticConfig &config, uint32_t level);
};

#endif /* INCLUDE_SOMA_UR5_HAPTIC_H_ */
