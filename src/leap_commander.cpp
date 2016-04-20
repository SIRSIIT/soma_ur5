#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <leap_motion/leapros.h>
#include <tf2/utils.h>

class LeapCom{
public:
LeapCom(){
    this->nh=new ros::NodeHandle();

    pub_robot_com=nh->advertise<geometry_msgs::PoseStamped>("/goal_pose",5);
    sub_pose= nh->subscribe("ee_pose", 1000, &LeapCom::robot_pose_callback, this);
    sub_leap= nh->subscribe("leapmotion/data", 1000, &LeapCom::leap_callback, this);

}
~LeapCom(){}
void leap_loop(){

}

protected:
ros::Publisher pub_robot_com;
ros::Subscriber sub_pose,sub_leap;
geometry_msgs::PoseStamped ee_pose,leap_pose;
ros::NodeHandle *nh;
tf2_ros::TransformBroadcaster tf_br;

void leap_callback(const leap_motion::leapros::ConstPtr &msg){
    leap_pose.pose.position.x=msg->palmpos.x/1000;
    leap_pose.pose.position.y=msg->palmpos.y/1000;
    leap_pose.pose.position.z=msg->palmpos.z/1000;

    tf2::Quaternion q;
    //q.setRPY(msg->ypr.x*M_PI/180,msg->ypr.y*M_PI/180,msg->ypr.z*M_PI/180);
    //q.setRPY(msg->ypr.x*M_PI/180,0,0);
    //q.setRPY(0,-atan2(msg->normal.x,-msg->normal.y),0);



    tf2::Vector3 c(msg->normal.x,msg->normal.y,msg->normal.z);
    tf2::Vector3 a(msg->direction.x,msg->direction.y,msg->direction.z);
    tf2::Vector3 b(c.cross(a));
    tf2::Matrix3x3 M(a.x(),b.x(),c.x(),
                     a.y(),b.y(),c.y(),
                     a.z(),b.z(),c.z());
    M.getRotation(q);
    //ROS_INFO("Det: %f", M.determinant());

    tf2::convert(q,leap_pose.pose.orientation);


    leap_pose.header.stamp=ros::Time::now();
    leap_pose.header.frame_id="leap_frame";

       geometry_msgs::TransformStamped trans;
       trans.header.frame_id="leap_frame";
       trans.child_frame_id="hand_frame";
       trans.transform.translation.x=leap_pose.pose.position.x;
       trans.transform.translation.y=leap_pose.pose.position.y;
       trans.transform.translation.z=leap_pose.pose.position.z;
       trans.transform.rotation=leap_pose.pose.orientation;


    tf_br.sendTransform(trans);
    pub_robot_com.publish(leap_pose);
}


void robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    ee_pose=*msg;
}

};


int main(int argc, char **argv){
    ros::init(argc, argv, "ur5_model");

    LeapCom *lc=new LeapCom();
    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        lc->leap_loop();
    }
    delete lc;

    return 0;
}
