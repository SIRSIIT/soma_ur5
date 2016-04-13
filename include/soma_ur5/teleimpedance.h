#ifndef INCLUDE_TELEIMPEDANCE_H_
#define INCLUDE_TELEIMPEDANCE_H_


#include <soma_ur5/grasp_feedback.h>

struct teleimp_options{
    double step_time;
    unsigned int window_size;
    double lambda,ns1,ns2,bias,D1,D2,Kte,Ktn;

};

class Teleimpedance : public Grasp_Feedback {
public:
    Teleimpedance(struct teleimp_options in) : Grasp_Feedback(in.window_size,in.step_time){
        update_opts(in);
    }
    void update_opts(teleimp_options in){
        options=in;
    }

    double t_motor,t_model;
    double getFeedback(){
        t_model=torque_model(q_avg,qd_avg)+options.bias;
        t_motor=torque_motor(cur);
        return t_motor-t_model;
    }
protected:
    struct teleimp_options options;

    double torque_motor(double current){
        return(options.Ktn*current);

    }

    double torque_model(double q_in, double qd_in){
        // From "Exploring Teleimpedance and Tactile Feedback for Intuitive Control of the Pisa/IIT SoftHand" Ajoudani et at, 2014 T-Hap
        // t_int = lambda/(s+lambda)*(K_tn*I_ref+lambda*J_n*q'-t_model)-lambda*J_n*q'

        // t_model=lambda/(s+lambda)*(K_tn*I_ref-lambda*J_n*q')
        // t_model= (1 + ns1)*K_te(q-q0)+D1q';
        //         (1 - ns2)*K_te(q-q0)-D2q';

        if(qd>=0){
            return (1+options.ns1)*options.Kte*(q_in)+options.D1*qd_in;
        }
        else{
            return (1+options.ns2)*options.Kte*(q_in)+options.D2*qd_in;
        }
    }

};


#endif //#define INCLUDE_TELEIMPEDANCE_H_

/*
    double fuzzy(double current){
        if(fabs(hand_pos_dot)>10000){
            fz_prev*=0.9;
            return fz_prev;
        }
        else{
            fz_prev=current;
            return current;
        }
    }
    double spring(double current){
        std_msgs::Float64MultiArray a;
        a.data.push_back(closure_ref);
        a.data.push_back(hand_pos/19000);

        pub_debug.publish(a);

        return (closure_ref-hand_pos/19000);
    }
    */
