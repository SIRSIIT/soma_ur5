struct teleimp_options{

    double lambda,ns1,ns2,bias,D1,D2,Kte,Ktn;

};

class Teleimpedance{
public:
    Teleimpedance(teleimp_options in){
        update_opts(in);
    }
    void update_opts(teleimp_options in){
        options=in;
    }
protected:
    teleimp_options options;

    double getFeedback(double q,double dq, double i){

    }


};
