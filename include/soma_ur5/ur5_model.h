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

typedef Eigen::Matrix< double, 6, 1 > Vector6d;

class UR5_Model{
public:
    UR5_Model();
    Vector6d getGravityTorques(Vector6d q);
    void run();
protected:
    ros::NodeHandle *nh;
    KDL::Tree robot_tree;
    KDL::Chain robot_chain;
    KDL::ChainJntToJacSolver *Jac_solver;
    KDL::RotationalInertia Cube_Rot_Inertia(double m,double w, double h, double d);
    std::array<int,6> jo;
    bool init,using_gazebo;
    ros::Subscriber sub_joints;
    ros::Publisher pub_joint_torque;
    sensor_msgs::JointState cur_joints;


    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void calculateJacobian(KDL::JntArray in);

};
