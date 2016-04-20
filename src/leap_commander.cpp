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

    geometry_msgs::TransformStamped get_hand_transform(leap_motion::leapros::ConstPtr msg){
        geometry_msgs::TransformStamped trans;
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

        tf2::convert(q,trans.transform.rotation);
        trans.header.frame_id="leap_frame";
        trans.child_frame_id="hand_frame";
        trans.header.stamp=ros::Time::now();
        trans.transform.translation.x=msg->palmpos.x/1000;
        trans.transform.translation.y=msg->palmpos.y/1000;
        trans.transform.translation.z=msg->palmpos.z/1000;
        return trans;

    }
    double get_hand_opening(leap_motion::leapros::ConstPtr msg){
        double opening;
        tf2::Vector3 mf_d(msg->middle_distal.x,msg->middle_distal.y,msg->middle_distal.z);
        tf2::Vector3 mf_t(msg->middle_tip.x,msg->middle_tip.y,msg->middle_tip.z);
        tf2::Vector3 h_d(msg->direction.x,msg->direction.y,msg->direction.z);
        opening=(mf_t-mf_d).angle(h_d)/M_PI;
        return opening;
    }



    geometry_msgs::PoseStamped transf2pose(geometry_msgs::TransformStamped in){
        geometry_msgs::PoseStamped pose;
        pose.header=in.header;
        pose.pose.position.x=in.transform.translation.x;
        pose.pose.position.y=in.transform.translation.y;
        pose.pose.position.z=in.transform.translation.z;
        //tf2::convert(in.transform.translation,pose.pose.position);
        tf2::convert(in.transform.rotation   ,pose.pose.orientation);
        return pose;
    }

    void leap_callback(const leap_motion::leapros::ConstPtr &msg){
        geometry_msgs::TransformStamped trans;
        double opening;
        trans=get_hand_transform(msg);
        tf_br.sendTransform(trans);
        leap_pose=transf2pose(trans);
        pub_robot_com.publish(leap_pose);
        opening=get_hand_opening(msg);
        ROS_INFO("opening: %f",opening);
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
