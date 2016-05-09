#include <soma_ur5/ur5_controller_main.h>


void UR5_Control_ROS::run(){
    while(ros::ok()){
        ros::spinOnce();
        ros::Rate(100).sleep();
    }
}

void UR5_Control_ROS::goal_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    trajectory_msgs::JointTrajectory vels;
    vels=ur5_model->calcSpeeds(ur5_model->getEEpose(),msg->pose,speed_gain);

    speed_command.publish(vels);
}

void UR5_Control_ROS::joint_update(const sensor_msgs::JointState &jnt){
    geometry_msgs::PoseStamped ee_pose;
    ur5_model->updateJoints(jnt);
    ee_pose.pose=ur5_model->getEEpose();
    ee_pose.header.stamp=ros::Time::now();
    ee_pose.header.frame_id="base_link";
    ROS_DEBUG("%f %f %f",ee_pose.pose.position.x,ee_pose.pose.position.y,ee_pose.pose.position.z);
    pub_ee_pose.publish(ee_pose);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ur5_controller");
    UR5_Control_ROS ur5_control;
    ur5_control.run();
}
