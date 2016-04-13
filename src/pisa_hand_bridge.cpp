#include <cstdio>
#include <ros/ros.h>
#include <qb_interface/handRef.h>
#include <qb_interface/handPos.h>
#include <qb_interface/handCurrent.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <soma_ur5/dyn_ur5_handConfig.h>
#include <soma_ur5/teleimpedance.h>

class Pisa_interface{
public:
    dynamic_reconfigure::Server<soma_ur5::dyn_ur5_handConfig> config_server;

    Pisa_interface(){
        nh=new ros::NodeHandle();

        pub_ffeed=nh->advertise<std_msgs::Float64>("grip_feedback",5);
        pub_grip=nh->advertise<qb_interface::handRef>("/qb_class/hand_ref",5);
        pub_debug=nh->advertise<std_msgs::Float64MultiArray>("teleimpedance",5);

        sub_grip= nh->subscribe("grip_cmd", 1000, &Pisa_interface::haptic_cb, this);
        sub_curr= nh->subscribe("/qb_class/hand_current", 1000, &Pisa_interface::hand_cb, this);
        sub_pedal= nh->subscribe("hap_pedal", 1000, &Pisa_interface::pedal_cb, this);
        sub_h_clos= nh->subscribe("/qb_class/hand_measurement", 1000, &Pisa_interface::hand_meas_cb, this);

        nh->getParam("step_time", ti_options.step_time);
        ti_options.window_size=20;
        ti=new Teleimpedance(ti_options);

        dynamic_reconfigure::Server<soma_ur5::dyn_ur5_handConfig>::CallbackType f;
        f=boost::bind(&Pisa_interface::config_cb, this, _1, _2);
        config_server.setCallback(f);
    }
    void run(){
        ros::spin();
    }

    void config_cb(soma_ur5::dyn_ur5_handConfig &config, uint32_t level) {
        ROS_DEBUG("Reconfigure Request.");
        grip_force_scale=config.grip_force_scale;
        ti_options.lambda=config.teleimp_lambda;
        ti_options.Kte=config.teleimp_K_te;
        ti_options.Ktn=config.teleimp_K_tn;
        ti_options.D1=config.teleimp_D1;
        ti_options.D2=config.teleimp_D2;
        ti_options.ns1=config.teleimp_ns1;
        ti_options.ns2=config.teleimp_ns2;
        ti_options.bias=config.teleimp_bias;
        ti->update_opts(ti_options);

    }

protected:
    ros::NodeHandle *nh;
    ros::Subscriber sub_grip,sub_curr,sub_pedal,sub_h_clos;
    ros::Publisher pub_grip,pub_ffeed,pub_debug;
    double grip_force_scale;
    struct teleimp_options ti_options;
    Teleimpedance *ti;

    bool pedal;
    double step_time;

    void pedal_cb(const std_msgs::Bool::ConstPtr &msg){
        pedal=msg->data;
    }

    void haptic_cb(const std_msgs::Float64::ConstPtr &msg){
        double closure_ref=(30-msg->data)*0.03; // 29->0 & 0->0.9
        if(pedal){
            qb_interface::handRef ref;
            ref.closure.push_back(closure_ref);
            pub_grip.publish(ref);
        }
    }
    void hand_meas_cb(const qb_interface::handPos::ConstPtr &msg){
        if(!isnan(msg->closure.at(0))){
            ti->update_pos(msg->closure.at(0));
        }
    }

    void debug_feedback(){

        std_msgs::Float64MultiArray a;
        a.data.push_back(ti->cur);
        a.data.push_back(ti->cur_avg);
        a.data.push_back(ti->q);
        a.data.push_back(ti->q_avg);
        a.data.push_back(ti->qd);
        a.data.push_back(ti->qd_avg);
        a.data.push_back(ti->t_model);
        a.data.push_back(ti->t_motor);
        a.data.push_back(ti->getFeedback());
        pub_debug.publish(a);
    }

    void hand_cb(const qb_interface::handCurrent::ConstPtr &msg){

        ti->update_cur(msg->current.at(0));

        if(pedal){
            std_msgs::Float64 cur;
            cur.data=ti->getFeedback()*grip_force_scale;
            pub_ffeed.publish(cur);
            debug_feedback();
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "pisa_hand_bridge");

    Pisa_interface pi;
    pi.run();

}
