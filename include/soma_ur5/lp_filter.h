#ifndef INCLUDE_SOMA_LP_FILTER_H_
#define INCLUDE_SOMA_LP_FILTER_H_

#include <vector>
#include <ctime>

class LP_Filter{
public:
    LP_Filter(int samples){
        t=0;
        full=false;
        n=samples;
        measurements.resize(samples);
    }
    int n;
    int t;
    bool full;
    std::vector<double> measurements;
    double filtered;
    clock_t last_sample;

    void add_measurement(double in){
        measurements.at(t)=in;
        last_sample = clock();
        t++;
        if(t==n) {
            full=true;
            t=0;
        }
    }

    double get_average(){
        double sum=0;
        int number;

        if(full) number=n;
        else number=t;

        for(int i=0;i<number;i++){
            sum+=measurements.at(i);
        }
        return sum/number;
    }

    double get_frequency(){
        clock_t now = clock();
        double elapsed_secs = double(now - last_sample) / CLOCKS_PER_SEC;
        return 1/elapsed_secs;
    }
};

#endif /* INCLUDE_SOMA_LP_FILTER_H_ */


