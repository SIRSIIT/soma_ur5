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
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <std_srvs/Empty.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>

typedef Eigen::Matrix< double, 6, 6 > Matrix6d;
typedef Eigen::Matrix< double, 6, 1 > Vector6d;

class Collisions{
public:
    Collisions(){
        initialize();
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
    ros::Publisher pub_F_ext,pub_F_ext_body;
    ros::NodeHandle *nh;
    ros::ServiceServer srv_ft_bias;
    sensor_msgs::JointState cur_joints;
    geometry_msgs::WrenchStamped cur_ft;
    tf2_ros::TransformListener *listener;
    tf2_ros::Buffer tf_buffer;
    geometry_msgs::Wrench ft_offset;
    KDL::Chain robot_chain;
    KDL::Tree robot_tree;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

    void initialize(){
        nh=new ros::NodeHandle();

        sub_torques= nh->subscribe("kdl_joints", 1000, &Collisions::joint_torque_callback, this);
        sub_ft_sensor= nh->subscribe("/netft_data", 1000, &Collisions::ft_sensor_callback, this);
        pub_F_ext=nh->advertise<geometry_msgs::WrenchStamped>("external_force",10);
        pub_F_ext_body=nh->advertise<geometry_msgs::WrenchStamped>("external_force_body",10);
        srv_ft_bias = nh->advertiseService("bias_ur5_weight",&Collisions::ft_bias_srv, this);

        while (cur_joints.effort.size()==0 || cur_ft.header.frame_id == ""){
            ros::spinOnce();
            ros::Rate(5).sleep();
        }

        listener=new tf2_ros::TransformListener(tf_buffer);

        ft_sensor_offset();

        kdl_parser::treeFromParam("/robot_description",robot_tree);
        robot_tree.getChain("base_link","ati_base",robot_chain);
        ROS_WARN_STREAM("robot_chain:" << robot_chain.getSegment(6).getName().c_str());
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain));
    }

    bool ft_bias_srv(std_srvs::Empty::Request &req,std_srvs::Empty::Response &rsp){
        ft_sensor_offset();
        return true;
    }

    void joint_torque_callback(const sensor_msgs::JointState::ConstPtr &msg){
        cur_joints=*msg;
    }
    void ft_sensor_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg){
        cur_ft=*msg;
    }

    void calculate_jac2(double cur_q[6], Matrix6d &J){
        KDL::Jacobian  J_;
        KDL::JntArray q_;
        J_.resize(6);
        q_.resize(6);
        for(int i=0;i<6;i++) q_(i)=cur_q[i];
        jnt_to_jac_solver_->JntToJac(q_, J_);
        //ROS_INFO("JACOBIAN KDL:");
        for(int i=0;i<6;i++) {
            for(int j=0;j<6;j++){
                J(i,j)=J_.data(i,j);
                //      printf("%f\t",J(i,j));
            }
            //   printf("\n");
        }

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
        //        ROS_INFO("JACOBIAN OFW:");
        //        for(int i=0;i<6;i++) {
        //            for(int j=0;j<6;j++){
        //                printf("%f\t",J(i,j));
        //            }
        //            printf("\n");
        //        }
        //ROS_DEBUG("Determinant(J): %f",J.determinant());
    }


    void ft_sensor_offset(){
        geometry_msgs::Wrench ee_weight=end_effector_weight();
        ft_offset=cur_ft.wrench;
        ft_offset.force.x-=ee_weight.force.x;
        ft_offset.force.y-=ee_weight.force.y;
        ft_offset.force.z-=ee_weight.force.z;

        ft_offset.torque.x-=ee_weight.torque.x;
        ft_offset.torque.y-=ee_weight.torque.y;
        ft_offset.torque.z-=ee_weight.torque.z;
        ROS_INFO("wh2: %f %f %f | %f %f %f",ee_weight.force.x,ee_weight.force.y,ee_weight.force.z,ee_weight.torque.x,ee_weight.torque.y,ee_weight.torque.z);

    }


    geometry_msgs::Wrench end_effector_weight(){
        double weight=0.9;
        double q[6];
        Eigen::Vector3d v_w(0,0,-9.8*weight);
        Eigen::Vector3d r_w(0,0,0.12);
        for (int i=0;i<6;i++){
            q[i]=cur_joints.position.at(i);
        }

        Eigen::Affine3d t_e=tf2::transformToEigen(fwd_kin_T(q,10));
        Eigen::Vector3d f_ee=t_e.inverse().rotation()*v_w;
        Eigen::Vector3d t_ee=r_w.cross(f_ee);

        geometry_msgs::Wrench ee_weight;
        ee_weight.force.x=f_ee[0];
        ee_weight.force.y=f_ee[1];
        ee_weight.force.z=f_ee[2];

        ee_weight.torque.x=t_ee[0];
        ee_weight.torque.y=t_ee[1];
        ee_weight.torque.z=t_ee[2];
        return ee_weight;
    }

    geometry_msgs::Wrench get_external_ee_force(){
        geometry_msgs::Wrench external,ee_weight;
        ee_weight=end_effector_weight();
        //cur_ft.wrench - ft_offset + end_effector_weight()

        external.force.x=cur_ft.wrench.force.x - ft_offset.force.x - ee_weight.force.x;
        external.force.y=cur_ft.wrench.force.y - ft_offset.force.y - ee_weight.force.y;
        external.force.z=cur_ft.wrench.force.z - ft_offset.force.z - ee_weight.force.z;

        external.torque.x=cur_ft.wrench.torque.x - ft_offset.torque.x - ee_weight.torque.x;
        external.torque.y=cur_ft.wrench.torque.y - ft_offset.torque.y - ee_weight.torque.y;
        external.torque.z=cur_ft.wrench.torque.z - ft_offset.torque.z - ee_weight.torque.z;

        ROS_DEBUG(" cur: %f %f %f | %f %f %f\n off: %f %f %f | %f %f %f\n wgh: %f %f %f | %f %f %f\n ext: %f %f %f | %f %f %f",
                  cur_ft.wrench.force.x,cur_ft.wrench.force.y,cur_ft.wrench.force.z,cur_ft.wrench.torque.x,cur_ft.wrench.torque.y,cur_ft.wrench.torque.z,
                  ft_offset.force.x,ft_offset.force.y,ft_offset.force.z,ft_offset.torque.x,ft_offset.torque.y,ft_offset.torque.z,
                  ee_weight.force.x,ee_weight.force.y,ee_weight.force.z,ee_weight.torque.x,ee_weight.torque.y,ee_weight.torque.z,
                  external.force.x,external.force.y,external.force.z,external.torque.x,external.torque.y,external.torque.z);
        return external;
    }

    geometry_msgs::TransformStamped fwd_kin_T(double q[6],int link_nr){
        geometry_msgs::PointStamped p;

        switch(link_nr){
        case 1:
            p.header.frame_id="base_link";
            p.point.z=0.05;
            break;
        case 2:
            p.header.frame_id="shoulder_link";
            p.point.z=0.05;
            break;
        case 3:
            p.header.frame_id="upper_arm_link";
            p.point.z=0.30;
            break;
        case 4:
            p.header.frame_id="forearm_link";
            p.point.z=0.25;
            break;
        case 5:
            p.header.frame_id="wrist_1_link";
            p.point.y=0.05;
            break;
        case 6:
            p.header.frame_id="wrist_2_link";
            p.point.z=0.05;
            break;
        case 7:
            p.header.frame_id="wrist_3_link";
            p.point.y=0.05;
            break;
        case 10:
            p.header.frame_id="ati_base_measurement";
            p.point.z=0.12;
        default:
            break;
        }

        geometry_msgs::TransformStamped base_to_frame;
        p.header.stamp=ros::Time::now();
        try{
            if(tf_buffer.canTransform("base_link",p.header.frame_id,p.header.stamp,ros::Duration(0.5))){
                p=tf_buffer.transform(p,"base_link");
                //base_to_frame=tf_buffer.lookupTransform("base_link",p.header.frame_id,p.header.stamp,ros::Duration(0.5));
                base_to_frame=tf_buffer.lookupTransform("base_link",ros::Time::now(),"ati_base_measurement",ros::Time::now(),"base_link",ros::Duration(0.5));
            }
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("Could not transform: %s", ex.what());
        }
        return base_to_frame;
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

        ROS_DEBUG("\nT: \n");
        for(int i=0;i<4;i++) {
            ROS_DEBUG("%f %f %f %f",T_j[i*4+0],T_j[i*4+1],T_j[i*4+2],T_j[i*4+3]);
        }
        ROS_DEBUG("\n");
        return fw;

    }
    Vector6d end_effector(){

        double q[6];
        Matrix6d Jac;
        Vector6d F_ee;
        geometry_msgs::WrenchStamped F_ext_ee;

        for (int i=0;i<6;i++) q[i]=cur_joints.position.at(i);

        F_ext_ee.header.frame_id="ati_base_measurement";
        F_ext_ee.header.stamp=ros::Time::now();
        F_ext_ee.wrench=get_external_ee_force();


        geometry_msgs::WrenchStamped F_ext;
        geometry_msgs::Vector3Stamped tmp,tmp2;


        tmp.header.frame_id="ati_base_measurement";
        tmp.header.stamp=ros::Time::now();

        if(tf_buffer.canTransform("base_link",tmp.header.frame_id,tmp.header.stamp,ros::Duration(0.5))){
            tmp.vector=F_ext_ee.wrench.force;
            tmp2=tf_buffer.transform(tmp,"base_link");
            F_ext.wrench.force=tmp2.vector;

            tmp.vector=F_ext_ee.wrench.torque;
            tmp2=tf_buffer.transform(tmp,"base_link");
            F_ext.wrench.torque=tmp2.vector;
        }


        F_ext.header.stamp=ros::Time::now();
        F_ext.header.frame_id="base_link";
        pub_F_ext.publish(F_ext);

        calculate_jac(q,Jac);
        //     ROS_INFO_STREAM("Jac1:\n"<<Jac << "\n");
        calculate_jac2(q,Jac);
        //     ROS_INFO_STREAM("Jac2:\n"<<Jac << "\n");

        fwd_kin(q);
        fwd_kin_T(q,4);

        F_ee << F_ext.wrench.force.x,
                F_ext.wrench.force.y,
                F_ext.wrench.force.z,
                F_ext.wrench.torque.x,
                F_ext.wrench.torque.y,
                F_ext.wrench.torque.z;



        Vector6d t_ee=Jac.transpose()*F_ee;

        ROS_INFO_STREAM(t_ee);
        return t_ee;

    }

    geometry_msgs::Wrench check_body(){
        Vector6d torques_from_ee,compensated_body_torques;
        torques_from_ee=end_effector();
        for (int i=0;i<6;i++) compensated_body_torques(i)=cur_joints.effort.at(i)-torques_from_ee(i);
        ROS_INFO_STREAM("BODY:\n" << compensated_body_torques << "\n");

    }

    void check_collisions(){

        geometry_msgs::WrenchStamped ee_force;
        ee_force.wrench=get_external_ee_force();
        ee_force.header.stamp=ros::Time::now();
        ee_force.header.frame_id="ati_base_measurement";

        geometry_msgs::WrenchStamped body_force;
        body_force.wrench=check_body();
        ee_force.header.stamp=ros::Time::now();
        ee_force.header.frame_id="base_link";


    }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "ur5_collisions");

    Collisions *col=new Collisions();

    col->run();
}

