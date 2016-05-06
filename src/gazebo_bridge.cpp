
/* gazebo_bridge.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: Joao Bimbo
 */

#include <soma_ur5/gazebo_bridge.h>


GazeboBridge::GazeboBridge(){
    this->nh=new ros::NodeHandle();
    //sub_comm = nh->subscribe("ur_driver/joint_speed", 1000, &GazeboBridge::vel_callback, this);
    sub_comm = nh->subscribe("ur_driver/joint_speed", 1000, &GazeboBridge::pos_callback, this);
    sub_joints = nh->subscribe("joint_states", 1000, &GazeboBridge::joint_callback, this);
    std::string str("vel_controller");
    for(int i=0;i<6;i++){
        pub_vels.push_back(nh->advertise<std_msgs::Float64>(str+std::to_string(i)+"/command",5));
    }

}

void GazeboBridge::joint_callback(const sensor_msgs::JointState::ConstPtr &msg){
    cur_joints=*msg;
}

void GazeboBridge::pos_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg){
    std_msgs::Float64 comm;
    if(msg->points.at(0).velocities.size()==6){
        for (size_t i=0;i<6;i++){
            comm.data=cur_joints.position.at(i)+0.001*msg->points.at(0).velocities.at(i);
            pub_vels.at(i).publish(comm);
        }
        last=ros::Time::now();
    }
    else ROS_ERROR("Invalid velocities array size");
}


void GazeboBridge::vel_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg){
    std_msgs::Float64 comm;
    if(msg->points.at(0).velocities.size()==6){
        for (size_t i=0;i<6;i++){
            comm.data=msg->points.at(0).velocities.at(i);
            pub_vels.at(i).publish(comm);
        }
        last=ros::Time::now();
    }
    else ROS_ERROR("Invalid velocities array size");
}



void GazeboBridge::run(){
    ros::Rate rate(200);

    while(ros::ok()){
        ros::spinOnce();
        /*if((ros::Time::now()-last).toSec()>0.1){
            for (size_t i=0;i<6;i++){
                std_msgs::Float64 comm;
                comm.data=0;
                pub_vels.at(i).publish(comm);
            }
        }*/
        rate.sleep();
    }
}


int main(int argc, char **argv){

    ros::init(argc, argv, "gazebo_bridge");
    GazeboBridge *gb=new GazeboBridge();

    gb->run();

    return 0;
}

