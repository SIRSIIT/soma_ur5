#ifndef INCLUDE_GRASP_FEEDBACK_H_
#define INCLUDE_GRASP_FEEDBACK_H_
#include <vector>
#include <numeric>

class Grasp_Feedback{
public:
    Grasp_Feedback(unsigned int n,double step_time_in) : WINDOW_SIZE(n), step_time(step_time_in){
        i_cu=0;
        i_dp=0;
        curr_hist.resize(WINDOW_SIZE);
        qd_hist.resize(WINDOW_SIZE);
        q_hist.resize(WINDOW_SIZE);
    }
    ~Grasp_Feedback() { }

    double step_time;
    double q,cur,qd,q_prev,q_avg,cur_avg,qd_avg;

    const unsigned int WINDOW_SIZE;
    std::vector<double> qd_hist,q_hist,curr_hist;
    int i_dp,i_cu;

    void update_pos(double q_in){
        q_prev=q;
        q=q_in;
        q_hist.at(i_dp%WINDOW_SIZE)=q_in;
        qd=(q-q_prev)/step_time;
        qd_hist.at(i_dp%WINDOW_SIZE)=qd;
        q_avg=average(q_hist);
        qd_avg=average(qd_hist);
        i_dp++;
    }
    void update_cur(double cur_in){
        if(cur_in!=0 && cur_in < 5000){ // Fix the flickering current
            cur=cur_in;
            curr_hist.at(i_cu%WINDOW_SIZE)=cur_in;
            i_cu++;
            cur_avg=average(curr_hist);
        }
    }

protected:
    double average(std::vector<double> vec){
        double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
        return(sum / curr_hist.size());
    }


};

#endif // INCLUDE_GRASP_FEEDBACK_H_
