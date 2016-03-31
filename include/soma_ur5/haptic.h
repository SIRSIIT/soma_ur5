#ifndef INCLUDE_SOMA_UR5_HAPTIC_H_
#define INCLUDE_SOMA_UR5_HAPTIC_H_


#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <soma_ur5/dhdc.h>
#include <soma_ur5/drdc.h>
#include <soma_ur5/utils.h>
#include <std_msgs/Float64.h>

class Haptic{

public:
    Haptic();
    ~Haptic();
    ros::NodeHandle *nh;
    bool haptic_loop();

private:
    ros::Subscriber sub_pose,sub_ft,sub_grip;
    ros::Publisher pub_hap_pose,pub_robot_com,pub_grip;
    geometry_msgs::PoseStamped ee_pose,hap_pose,hap_pose_initial,ee_pose_initial;

    double scale_factor;
    bool pedal_on;
    double grip_val;

    int initialize_haptic();
    bool GetHapticInfo(geometry_msgs::Pose &h_pose);
    bool SetHaptic();
    void robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void grip_callback(const std_msgs::Float64::ConstPtr &msg);
    bool move_haptic(geometry_msgs::Pose);
    bool goto_initial();
    geometry_msgs::Pose diff_pose(geometry_msgs::Pose);
    geometry_msgs::Pose scale_pose(geometry_msgs::Pose in, std::string mode);
};

#endif /* INCLUDE_SOMA_UR5_HAPTIC_H_ */
