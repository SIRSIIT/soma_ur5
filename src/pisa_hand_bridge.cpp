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

#define WINDOW_SIZE 5
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


        //        nh->getParam("hand/grip_force_scale", grip_force_scale);
        nh->getParam("step_time", step_time);
        dynamic_reconfigure::Server<soma_ur5::dyn_ur5_handConfig>::CallbackType f;
        f=boost::bind(&Pisa_interface::config_cb, this, _1, _2);
        config_server.setCallback(f);
        i_cu=0;
        i_dp=0;
        h_dot_hist.resize(WINDOW_SIZE);
        curr_hist.resize(WINDOW_SIZE);
        f_filtered.resize(WINDOW_SIZE);


    }
    void run(){
        ros::spin();
    }

    void config_cb(soma_ur5::dyn_ur5_handConfig &config, uint32_t level) {
        ROS_DEBUG("Reconfigure Request.");
        grip_force_scale=config.grip_force_scale;
        ti_lambda=config.teleimp_lambda;
        ti_Kte=config.teleimp_K_te;
        ti_Ktn=config.teleimp_K_tn;
        ti_D1=config.teleimp_D1;
        ti_ns=config.teleimp_ns;
    }

protected:
    ros::NodeHandle *nh;
    ros::Subscriber sub_grip,sub_curr,sub_pedal,sub_h_clos;
    ros::Publisher pub_grip,pub_ffeed,pub_debug;
    double grip_force_scale;
    bool pedal;
    double hand_pos_prev,hand_pos,hand_pos_dot;
    double ti_lambda,ti_ns,ti_D1,ti_Kte,ti_Ktn;
    double step_time;
    int i_dp,i_cu;
    std::vector<double> h_dot_hist,curr_hist,f_filtered;

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
    void hand_meas_cb(const qb_interface::handPos::ConstPtr &msg){
        if(!isnan(msg->closure.at(0))){
            hand_pos_prev=hand_pos;
            hand_pos=(double) msg->closure.at(0);
            double hand_pos_dot_tmp=(hand_pos-hand_pos_prev)/step_time;
            h_dot_hist.at(i_dp%WINDOW_SIZE)=hand_pos_dot_tmp;  i_dp++;
            double sum = std::accumulate(h_dot_hist.begin(), h_dot_hist.end(), 0.0);
            hand_pos_dot = sum / h_dot_hist.size();
        }
    }

    double signnum_c(double x) {
        if (x > 0.0) return 1.0;
        if (x < 0.0) return -1.0;
        return x;
    }

    double teleimpedance(double current){


        // From "Exploring Teleimpedance and Tactile Feedback for Intuitive Control of the Pisa/IIT SoftHand" Ajoudani et at, 2014 T-Hap
        // t_int = lambda/(s+lambda)*(K_tn*I_ref+lambda*J_n*q'-t_model)-lambda*J_n*q'
        // t_model=lambda/(s+lambda)*(K_tn*I_ref-lambda*J_n*q')

        //t_model= (1 +- ns)*K_te(q-q0)+D1q';

        std_msgs::Float64MultiArray a;
        double t_model=(1+(this->signnum_c(hand_pos_dot)*ti_ns))*ti_Kte*(hand_pos)+ti_D1*hand_pos_dot;
        double t_int=ti_Ktn*current-t_model;

        a.data.push_back(current);
        a.data.push_back(hand_pos_dot);
        a.data.push_back(t_model);
        a.data.push_back(t_int);
        pub_debug.publish(a);
        return t_int;
    }

    void hand_cb(const qb_interface::handCurrent::ConstPtr &msg){

        curr_hist.at(i_cu%WINDOW_SIZE)=(double) msg->current.at(0);
        double sum = std::accumulate(curr_hist.begin(), curr_hist.end(), 0.0);
        double avg_cur= sum / curr_hist.size();

        f_filtered.at(i_cu%WINDOW_SIZE)=teleimpedance(avg_cur);
        sum = std::accumulate(f_filtered.begin(), f_filtered.end(), 0.0);
        double avg_for= sum / f_filtered.size();


        i_cu++;
        if(pedal){
            std_msgs::Float64 cur;
            cur.data=avg_for*grip_force_scale;
            //cur.data=avg_cur*grip_force_scale*0.01+ti_lambda;
            pub_ffeed.publish(cur);
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "pisa_hand_bridge");

    Pisa_interface pi;
    pi.run();

}
