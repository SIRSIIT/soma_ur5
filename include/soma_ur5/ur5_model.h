#ifndef INCLUDE_SOMA_UR5_MODEL_H_
#define INCLUDE_SOMA_UR5_MODEL_H_


#include <ros/ros.h>
#include <urdf/model.h>
#include <Eigen/Core>
#include <kdl_conversions/kdl_msg.h>
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
#include <sirsiit_utils/lp_filter.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sirsiit_utils/utils.h>
#include <soma_ur5/dyn_ur5_modelConfig.h>
//#include <soma_ur5/dyn_ur5_modelParameters.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/action_server.h>
#include <soma_ur5/SOMAFrameworkAction.h>

typedef Eigen::Matrix< double, 6, 1 > Vector6d;

template <typename T>
inline bool const empty_msg(T a){
    bool ret=true;
    if(a.x!=0) ret=false;
    else if(a.y!=0) ret=false;
    else if(a.z!=0) ret=false;
    return ret;
}

class UR5_Model{
public:
    UR5_Model(ros::NodeHandle nh_in);
    void run();
protected:
    ros::NodeHandle *nh;
    KDL::Tree robot_tree,robot_tree_w_hand;
    KDL::Chain robot_chain,robot_chain_w_hand;
    KDL::ChainJntToJacSolver *Jac_solver;
    KDL::ChainIkSolverPos_LMA *inv_solver;
    KDL::ChainFkSolverPos_recursive *fksolv;
    KDL::RotationalInertia Cube_Rot_Inertia(double m,double w, double h, double d);
    std::array<int,6> jo;
    bool init,using_gazebo,using_hand;
    ros::Subscriber sub_joints,sub_goal_pose, sub_forces;
    ros::Publisher pub_joint_torque,pub_joint_kdl,pub_kdl_pose,speed_command;
    actionlib::ActionServer<soma_ur5::SOMAFrameworkAction> *act_srv;
    sensor_msgs::JointState cur_joints;
    Eigen::Matrix<double,6,2> currents_to_torques;
    std::vector<LP_Filter> cur_filters;
    geometry_msgs::WrenchStamped cur_force;


    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void force_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    bool calculateJacobian(KDL::JntArray in, Matrix6d &J);
    geometry_msgs::Pose getEEpose(KDL::JntArray joint_pos);
    KDL::JntArray getGravityTorques(KDL::JntArray q);
    void goal_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    trajectory_msgs::JointTrajectory calcSpeeds(geometry_msgs::Pose cur_pose, geometry_msgs::Pose goal_pose,double gain);
    Vector6d getDeltaX(geometry_msgs::Pose cur,geometry_msgs::Pose goal);
    trajectory_msgs::JointTrajectory  safety_enforcer( trajectory_msgs::JointTrajectory in);
    Vector6d fwd_kin(double q[6]);
    Matrix6d getJacobian(sensor_msgs::JointState j);

    std::map<std::string,double> map_j_lim,map_ws_lim;
    std::string control_topic;
    double max_angle,max_speed;
    trajectory_msgs::JointTrajectoryPoint prev_vel;

    //    soma_ur5::dyn_ur5_modelParameters *params_;
    dynamic_reconfigure::Server<soma_ur5::dyn_ur5_modelConfig> config_server;
    void reconfigureRequest(soma_ur5::dyn_ur5_modelConfig& config, uint32_t level);
    ros::Time t_last_command;
    bool stopped;
    int parse_goal(soma_ur5::SOMAFrameworkGoal::ConstPtr g );
    
    void execute_action(actionlib::ActionServer<soma_ur5::SOMAFrameworkAction>::GoalHandle goal, actionlib::ActionServer<soma_ur5::SOMAFrameworkAction>* as);
    void cancel_action(actionlib::ActionServer<soma_ur5::SOMAFrameworkAction>::GoalHandle goal, actionlib::ActionServer<soma_ur5::SOMAFrameworkAction>* as);
    bool monitor_wrench( soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb);
    bool monitor_pose( soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb);
    bool monitor_dummy( soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb);
    void control_force( soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb );
    void control_position( soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb);
    void control_velocity(  soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb );
    void control_follow( soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb);
    struct Params{
        double speed_gain;
    } params;

};
#endif /* INCLUDE_SOMA_UR5_MODEL_H_ */
