
/* gazebo_bridge.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: Joao Bimbo
 */

#include <soma_ur5/gazebo_bridge.h>


GazeboBridge::GazeboBridge(){
    this->nh=new ros::NodeHandle();
    sub_comm = nh->subscribe("ur_driver/joint_speed", 1000, &GazeboBridge::vel_callback, this);
    //sub_comm = nh->subscribe("ur_driver/joint_speed", 1000, &GazeboBridge::pos_callback, this);
    sub_joints = nh->subscribe("joint_states", 1000, &GazeboBridge::joint_callback, this);
    std::string str("vel_controller");
    for(int i=0;i<6;i++){
        pub_vels.push_back(nh->advertise<std_msgs::Float64>(str+std::to_string(i)+"/command",5));
    }
    position_mode=true;
    velocity_mode_before=true;
    started=false;using_gazebo=false;
    cur_joints.name.resize(6);cur_joints.position.resize(6);
    cur_joints.velocity.resize(6);cur_joints.effort.resize(6);
    while(!started){
        ros::spinOnce();
        ros::Rate(20).sleep();
    }
    jp_to_keep=cur_joints;
}

void GazeboBridge::joint_callback(const sensor_msgs::JointState::ConstPtr &msg){
    if(!started){
        jo.resize(joint_names.size());
        for (int i=0;i<jo.size();i++){
            for (int j=0;j<msg->name.size();j++){
                if(msg->name.at(j)==joint_names.at(i)){
                    jo[i]=j;
                }
            }
        }
        if(boost::iequals(msg->name.at(0),"elbow_joint")) using_gazebo=true;
        else using_gazebo=false;
        started=true;
    }
    cur_joints.header=msg->header;
    for(int i=0;i<6;i++){
        cur_joints.name.at(i)=msg->name.at(jo[i]);
        cur_joints.position.at(i)=msg->position.at(jo[i]);
        cur_joints.velocity.at(i)=msg->velocity.at(jo[i]);
        cur_joints.effort.at(i)=msg->effort.at(jo[i]);
    }
}

void GazeboBridge::keep_position(){
    std_msgs::Float64 comm;
    double Kp=12.0;
    for (size_t i=0;i<6;i++){
        //ROS_INFO("J%d : %f - %f",i,jp_to_keep.position.at(i),cur_joints.position.at(i));
        comm.data=Kp*(jp_to_keep.position.at(i)-cur_joints.position.at(i));
        pub_vels.at(i).publish(comm);
    }
}


void GazeboBridge::pos_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg){
    std_msgs::Float64 comm;
    position_mode= std::all_of(msg->points.at(0).velocities.begin(), msg->points.at(0).velocities.end(), [](int i) { return i==0; });

    if(position_mode){
        if(velocity_mode_before){
            jp_to_keep=cur_joints;
            velocity_mode_before=false;
        }
    }
    else {
        velocity_mode_before=true;
        if(msg->points.at(0).velocities.size()==6){
            for (size_t i=0;i<6;i++){
                comm.data=cur_joints.position.at(i)+0.1*msg->points.at(0).velocities.at(i);
                pub_vels.at(i).publish(comm);
            }
            last=ros::Time::now();
        }
        else ROS_ERROR("Invalid velocities array size");
    }

}


void GazeboBridge::vel_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg){
    std_msgs::Float64 comm;
    position_mode=true;
    //position_mode= std::all_of(msg->points.at(0).velocities.begin(), msg->points.at(0).velocities.end(), [](int i) { return i==0; });
    for (int i=0;i<msg->points.at(0).velocities.size();i++){
        if(msg->points.at(0).velocities.at(i)!=0) position_mode=false;
    }
    jp_to_keep=cur_joints;

    if(msg->points.at(0).velocities.size()==6){
        for (size_t i=0;i<6;i++){
            comm.data=10*msg->points.at(0).velocities.at(i);
            pub_vels.at(i).publish(comm);
        }
        last=ros::Time::now();
    }
    else ROS_ERROR("Invalid velocities array size");

}



void GazeboBridge::run(){
    ros::Rate rate(100);

    while(ros::ok()){
        ros::spinOnce();
        if((ros::Time::now()-last).toSec()>0.1 & !position_mode){
            ROS_INFO("Switching to pos mode");
            position_mode=true;

        }

        if(position_mode) {
            //   ROS_WARN("Keeping position");
            keep_position();
        }


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

