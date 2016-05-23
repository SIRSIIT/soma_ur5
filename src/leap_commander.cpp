#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <leap_motion/leapros.h>
#include <tf2/utils.h>
#include <keyboard/Key.h>
#include <soma_ur5/utils.h>
#include <std_msgs/Float64.h>
#include <qb_interface/handRef.h>

class LeapCom{
public:
    LeapCom(){
        this->nh=new ros::NodeHandle();

        pub_robot_com=nh->advertise<geometry_msgs::PoseStamped>("goal_pose",5);
        pub_gripper=nh->advertise<qb_interface::handRef>("/qb_class/hand_ref",5);
        sub_pose= nh->subscribe("ee_pose", 1000, &LeapCom::robot_pose_callback, this);
        sub_leap= nh->subscribe("leapmotion/data", 1000, &LeapCom::leap_callback, this);
        sub_keydown=nh->subscribe("/keyboard/keydown",1000, &LeapCom::keydown_callback,this);
        sub_keyup=nh->subscribe("/keyboard/keyup",1000, &LeapCom::keyup_callback,this);
        key_space_on=false;
        key_zero_on=false;
        scale_factor=1;
        BASE_FRAME=true;

    }
    ~LeapCom(){ }
    void leap_loop(){
        geometry_msgs::PoseStamped h_pose,com_pose;
        geometry_msgs::Pose d_pose;
        if(key_space_on || key_zero_on){

            if(BASE_FRAME){
                h_pose=leap_pose;
                d_pose=scale_pose(diff_pose(h_pose.pose),"h2r");
                tf2::Quaternion q_diff((utils::Pose2Transform(d_pose)*utils::Pose2Transform(ee_pose_initial.pose)).getRotation() );
                tf2::convert(q_diff, com_pose.pose.orientation);
                com_pose.pose.position.x=ee_pose_initial.pose.position.x+d_pose.position.x;
                com_pose.pose.position.y=ee_pose_initial.pose.position.y+d_pose.position.y;
                com_pose.pose.position.z=ee_pose_initial.pose.position.z+d_pose.position.z;

            }
            else{
                h_pose=leap_pose;
                d_pose=scale_pose(diff_pose(h_pose.pose),"h2r");
                com_pose.pose=utils::Transform2Pose(utils::Pose2Transform(ee_pose_initial.pose)*utils::Pose2Transform(d_pose));
            }

            com_pose.header.stamp=ros::Time::now();
            com_pose.header.frame_id="base_link";
            pub_robot_com.publish(com_pose);

            qb_interface::handRef opening_msg;
            opening_msg.closure.push_back(opening);
            pub_gripper.publish(opening_msg);
        }
    }

protected:
    ros::Publisher pub_robot_com,pub_gripper;
    ros::Subscriber sub_pose,sub_leap,sub_keydown,sub_keyup;
    geometry_msgs::PoseStamped ee_pose,leap_pose,ee_pose_initial,leap_pose_initial;
    ros::NodeHandle *nh;
    tf2_ros::TransformBroadcaster tf_br;
    leap_motion::leapros hand_state;
    bool key_space_on,key_zero_on;
    double scale_factor,opening;
    bool BASE_FRAME;


    geometry_msgs::TransformStamped get_hand_transform(leap_motion::leapros::ConstPtr msg){
        geometry_msgs::TransformStamped trans;
        tf2::Quaternion q;
        //q.setRPY(msg->ypr.x*M_PI/180,msg->ypr.y*M_PI/180,msg->ypr.z*M_PI/180);
        //q.setRPY(msg->ypr.x*M_PI/180,0,0);
        //q.setRPY(0,-atan2(msg->normal.x,-msg->normal.y),0);

        tf2::Vector3 c(-msg->normal.x,-msg->normal.y,-msg->normal.z);
        tf2::Vector3 a(msg->direction.x,msg->direction.y,msg->direction.z);
        tf2::Vector3 b(c.cross(a));
        tf2::Matrix3x3 M(a.x(),b.x(),c.x(),
                         a.y(),b.y(),c.y(),
                         a.z(),b.z(),c.z());
        M.getRotation(q);
        ROS_INFO_THROTTLE(1,"Det: %f", M.determinant());

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
        geometry_msgs::TransformStamped trans,hand_in_leapbase,leap_base;
        tf2::Transform hand_in_leapbase_tf,trans_tf,leap_base_tf;

        trans=get_hand_transform(msg);
        tf_br.sendTransform(trans);
        tf2::convert(trans.transform,trans_tf);
        leap_base_tf=tf2::Transform(tf2::Quaternion(-M_PI/2,0,-M_PI/2),tf2::Vector3(0,0,0.01));
        tf2::convert(leap_base_tf,leap_base.transform);
        leap_base.child_frame_id="leap_frame";
        leap_base.header.frame_id="leap_base";
        leap_base.header.stamp=ros::Time::now();
        tf_br.sendTransform(leap_base);

        hand_in_leapbase_tf=leap_base_tf*trans_tf;
        tf2::convert(hand_in_leapbase_tf,hand_in_leapbase.transform);
        hand_in_leapbase.header.frame_id="leap_base";
        hand_in_leapbase.header.stamp=trans.header.stamp;
        //tf_br.sendTransform(hand_in_leapbase);

       leap_pose=transf2pose(hand_in_leapbase); //WORKING before
     //   leap_pose=transf2pose(trans);  //Trial
        hand_state=*msg;
        opening=get_hand_opening(msg)*1.0;
        //     pub_robot_com.publish(leap_pose);

    }
    void robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
        ee_pose=*msg;
    }

    void keydown_callback(const keyboard::Key::ConstPtr &msg){
        if(msg->code==msg->KEY_KP0){
            key_space_on=false;
            key_zero_on=true;
            BASE_FRAME=true;
            leap_pose_initial=leap_pose;
            ee_pose_initial=ee_pose;
            ROS_WARN("Working on Base frame now");

        }
        else if(msg->code==msg->KEY_SPACE)    {
            key_space_on=true;
            BASE_FRAME=false;
            leap_pose_initial=leap_pose;
            ee_pose_initial=ee_pose;
            ROS_INFO("ON");
        }
    }
    void keyup_callback(const keyboard::Key::ConstPtr &msg){
        if(msg->code==msg->KEY_KP0){
            key_zero_on=false;
            BASE_FRAME=false;
        }
        if(msg->code==msg->KEY_SPACE)     {
            key_space_on=false;
            ROS_INFO("OFF");
        }
    }


    geometry_msgs::Pose diff_pose(geometry_msgs::Pose in){
        geometry_msgs::Pose tmp;
        /*
        tf2::Quaternion q1,q2;
        tf2::convert(leap_pose_initial.pose.orientation,q1);
        tf2::convert(in.orientation,q2);
        tf2::convert(q2*q1.inverse(),tmp.orientation);

        tmp.position.x=in.position.x-leap_pose_initial.pose.position.x;
        tmp.position.y=in.position.y-leap_pose_initial.pose.position.y;
        tmp.position.z=in.position.z-leap_pose_initial.pose.position.z;
  */

        tf2::Transform li,lc;
        li=utils::Pose2Transform(leap_pose_initial.pose);
        lc=utils::Pose2Transform(in);
        tmp=utils::Transform2Pose(li.inverse()*lc);
        return tmp;
    }
    geometry_msgs::Pose scale_pose(geometry_msgs::Pose in,std::string mode){
        geometry_msgs::Pose out;

        if(mode=="r2h"){
            out.position.x=in.position.x*scale_factor;
            out.position.y=in.position.y*scale_factor;
            out.position.z=in.position.z*scale_factor;
        }
        else if(mode=="h2r"){
            out.position.x=in.position.x/scale_factor;
            out.position.y=in.position.y/scale_factor;
            out.position.z=in.position.z/scale_factor;
        }

        out.orientation=in.orientation;

        return out;
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
