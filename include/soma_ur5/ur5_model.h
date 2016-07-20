#ifndef INCLUDE_SOMA_UR5_MODEL_H_
#define INCLUDE_SOMA_UR5_MODEL_H_


#include <ros/ros.h>
#include <urdf/model.h>
#include <Eigen/Core>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/articulatedbodyinertia.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <array>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <soma_ur5/lp_filter.h>
#include <geometry_msgs/PoseStamped.h>

typedef Eigen::Matrix< double, 6, 1 > Vector6d;

class UR5_Model{
public:
    UR5_Model();
    Vector6d getGravityTorques(Vector6d q);
    void run();
protected:
    ros::NodeHandle *nh;
    KDL::Tree robot_tree,robot_tree_w_hand;
    KDL::Chain robot_chain,robot_chain_w_hand;
    KDL::ChainJntToJacSolver *Jac_solver;
    KDL::RotationalInertia Cube_Rot_Inertia(double m,double w, double h, double d);
    std::array<int,6> jo;
    bool init,using_gazebo,using_hand;
    ros::Subscriber sub_joints;
    ros::Publisher pub_joint_torque,pub_joint_kdl,pub_kdl_pose;
    sensor_msgs::JointState cur_joints;
    Eigen::Matrix<double,6,2> currents_to_torques;
    std::vector<LP_Filter> cur_filters;


    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void calculateJacobian(KDL::JntArray in);

};
#endif /* INCLUDE_SOMA_UR5_MODEL_H_ */
