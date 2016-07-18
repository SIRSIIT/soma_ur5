#include <cstdio>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <ur_kinematics/ur_kin.h>
#include <Eigen/Core>
#include <soma_ur5/utils.h>

typedef Eigen::Matrix< double, 6, 6 > Matrix6d;
typedef Eigen::Matrix< double, 6, 1 > Vector6d;

class Collisions{
public:
    Collisions(){
        ROS_INFO("0ah");

        initialize();
        ROS_INFO("ag0");

    }

    void run(){
        ros::Rate rate(20);
        while(ros::ok()){
            ros::spinOnce();
            check_collisions();
            rate.sleep();
        }
    }

protected:
    ros::Subscriber sub_ft_sensor,sub_torques;
    ros::NodeHandle *nh;
    sensor_msgs::JointState cur_joints;
    geometry_msgs::WrenchStamped cur_ft;


    void initialize(){
        nh=new ros::NodeHandle();
        ROS_INFO("02541");

        sub_torques= nh->subscribe("kdl_joints", 1000, &Collisions::joint_torque_callback, this);
        sub_ft_sensor= nh->subscribe("/netft_data", 1000, &Collisions::ft_sensor_callback, this);
        ROS_INFO("afsas0");

    }

    void joint_torque_callback(const sensor_msgs::JointState::ConstPtr &msg){
        cur_joints=*msg;
    }
    void ft_sensor_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg){
        cur_ft=*msg;
    }

    void calculate_jac(double cur_q[6], Matrix6d &J){

        double h=0.001;
        double next_q[6];

        Vector6d cur_c,nc;

        cur_c=fwd_kin(cur_q);

        for(int i=0;i<6;i++) {
            for(int j=0;j<6;j++) {
                next_q[j]=cur_q[j];
            }
            next_q[i]+=h;
            nc=fwd_kin(next_q);
            J.col(i) = 1/h*(nc-cur_c).col(0);
        }
        //ROS_DEBUG("Determinant(J): %f",J.determinant());
    }

    Vector6d fwd_kin(double q[6]){
        double r,p,y;
        Vector6d fw;
        double T_j[16];
        ur_kinematics::forward(q,T_j);
        tf2::Matrix3x3(T_j[0],T_j[1],T_j[2],
                T_j[4],T_j[5],T_j[6],
                T_j[8],T_j[9],T_j[10]).getRPY(r,p,y);
        fw << T_j[3], T_j[7], T_j[11], r, p, y;
        return fw;

    }
    void end_effector(){

        ROS_INFO("0");

        double q[6];
        Matrix6d Jac;
        Vector6d F_ee;
        ROS_INFO("1");

        for (int i=0;i<6;i++) q[i]=cur_joints.position.at(i);

        ROS_INFO("2");
        calculate_jac(q,Jac);
        ROS_INFO("3");

        F_ee << cur_ft.wrench.force.x,
                cur_ft.wrench.force.y,
                cur_ft.wrench.force.z,
                cur_ft.wrench.torque.x,
                cur_ft.wrench.torque.y,
                cur_ft.wrench.torque.z;

        Vector6d t_ee=utils::pseudoinv(Jac)*F_ee;
        ROS_INFO_STREAM(t_ee);


    }

    void check_collisions(){
        ROS_INFO("04");

        end_effector();
        ROS_INFO("30");


    }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "ur5_model");
    ROS_INFO("0555");

    Collisions *col=new Collisions();

    ROS_INFO("10");

    col->run();
}

