#include <cstdio>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <soma_ur5/SOMAFrameworkAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

class SOMA_Framework{

public:
    SOMA_Framework(std::string name) :
      as_(nh_, name, boost::bind(&SOMA_Framework::executeCB, this, _1), false),
      action_name_(name)
    {
      as_.start();
      state=0;
      pose_pub=nh_.advertise<geometry_msgs::PoseStamped>("goal_pose",2);
      pose_sub=nh_.subscribe("ee_pose",1,&SOMA_Framework::pose_callback,this);
    }

protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<soma_ur5::SOMAFrameworkAction> as_;
    std::string action_name_;
    soma_ur5::SOMAFrameworkFeedback feedback_;
    soma_ur5::SOMAFrameworkResult result_;
    ros::Publisher pose_pub;
    ros::Subscriber pose_sub;
    geometry_msgs::PoseStamped cur_pose;
    int state;

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
        cur_pose=*msg;
    }

    bool wrench_similar(geometry_msgs::Wrench w1,geometry_msgs::Wrench w2){
        geometry_msgs::Wrench dp;
        dp.force.x=w1.force.x-w2.force.x;
        dp.force.y=w1.force.y-w2.force.y;
        dp.force.z=w1.force.z-w2.force.z;

        dp.torque.x=w1.torque.x-w2.torque.x;
        dp.torque.y=w1.torque.y-w2.torque.y;
        dp.torque.z=w1.torque.z-w2.torque.z;

        double dt=sqrt(dp.force.x*dp.force.x+dp.force.y*dp.force.y+dp.force.z*dp.force.z);
        double dr=sqrt(dp.torque.x*dp.torque.x+dp.torque.y*dp.torque.y+dp.torque.z*dp.torque.z);

        if(dt<0.01 && dr<0.1) return true;
        else return false;
    }

    bool pose_similar(geometry_msgs::Pose p1,geometry_msgs::Pose p2){
        double dr,dt;
        geometry_msgs::Point dp;

        dp.x=p1.position.x-p2.position.x;
        dp.y=p1.position.y-p2.position.y;
        dp.z=p1.position.z-p2.position.z;

        tf2::Quaternion q1=tf2::Quaternion(p1.orientation.x,p1.orientation.y,p1.orientation.z,p1.orientation.w);
        tf2::Quaternion q2=tf2::Quaternion(p2.orientation.x,p2.orientation.y,p2.orientation.z,p2.orientation.w);


        dr=tf2::angle(q1,q2);
        dt=sqrt(dp.x*dp.x+dp.y*dp.y+dp.z*dp.z);
        ROS_INFO("DR: %f, DT: %f",dr,dt);
        if(dt<0.01 && dr<0.1) return true;
        else return false;

    }

    bool goto_pose(soma_ur5::SOMAFrameworkGoalConstPtr goal){
        geometry_msgs::PoseStamped des_pose;
        des_pose.pose=goal->pose;
        des_pose.header.frame_id="base_link";
        des_pose.header.stamp=ros::Time::now();
        pose_pub.publish(des_pose);
        if(pose_similar(cur_pose.pose,des_pose.pose)) return true;
        else return false;
    }
    bool move_twist(soma_ur5::SOMAFrameworkGoalConstPtr goal){
        return true;

    }
    bool close_hand(soma_ur5::SOMAFrameworkGoalConstPtr goal){
        return false;
    }


    void executeCB(const soma_ur5::SOMAFrameworkGoalConstPtr &goal)
    {
        ros::Time t_start=ros::Time::now();
        ros::Rate rate(10);
        state=1;
        while (state!=999 && state !=5){

            switch(state){
            case 1:
                if(goto_pose(goal)) state=2;
                break;
            case 2:
                if(move_twist(goal)) state=3;
                break;
            case 3:
                if(close_hand(goal)) state=5;
                break;
            case 5:
            default:
                break;

            }
            ROS_INFO("state: %d",state);
            feedback_.cur_pose=cur_pose.pose;
            feedback_.header.stamp=ros::Time::now();
            as_.publishFeedback(feedback_);
            if((ros::Time::now()-t_start).toSec()>goal->max_duration) state=999;
            ros::spinOnce();
            rate.sleep();
        }
        result_.success=true;
        as_.setSucceeded(result_);
        return;
    }
};



int main(int argc, char **argv){
    ros::init(argc, argv, "soma_framework");


    SOMA_Framework a(ros::this_node::getName());
    ros::spin();

	return 0;
}
