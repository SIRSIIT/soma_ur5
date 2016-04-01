#include <cstdio>
#include <ros/ros.h>
#include <qb_interface/handRef.h>
#include <qb_interface/handCurrent.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

class Pisa_interface{
public:
    Pisa_interface(){
        nh=new ros::NodeHandle();

        pub_ffeed=nh->advertise<std_msgs::Float64>("grip_feedback",5);
        pub_grip=nh->advertise<qb_interface::handRef>("/qb_class/hand_ref",5);

        sub_grip= nh->subscribe("grip_cmd", 1000, &Pisa_interface::haptic_cb, this);
        sub_curr= nh->subscribe("/qb_class/hand_current", 1000, &Pisa_interface::hand_cb, this);
        sub_pedal= nh->subscribe("hap_pedal", 1000, &Pisa_interface::pedal_cb, this);


        nh->getParam("grip_force_scale", grip_force_scale);
    }
    void run(){
        ros::spin();
    }

protected:
    ros::NodeHandle *nh;
    ros::Subscriber sub_grip,sub_curr,sub_pedal;
    ros::Publisher pub_grip,pub_ffeed;
    double grip_force_scale;
    bool pedal;

    void pedal_cb(const std_msgs::Bool::ConstPtr &msg){
        pedal=msg->data;
    }

    void haptic_cb(const std_msgs::Float64::ConstPtr &msg){

        if(pedal){
            qb_interface::handRef ref;
            ref.closure.push_back((30-msg->data)/30);
            pub_grip.publish(ref);
        }
    }

    void hand_cb(const qb_interface::handCurrent::ConstPtr &msg){
        if(pedal){
            std_msgs::Float64 cur;
            cur.data=((double) msg->current.at(0))*grip_force_scale;
            pub_ffeed.publish(cur);
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "pisa_hand_bridge");

    Pisa_interface pi;
    pi.run();

}
