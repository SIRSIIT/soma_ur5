//
// Created by enrico on 10/09/2020.
//

#include <soma_ur5/haptic_guidance.h>


haptic_guidance::haptic_guidance(){
    //this->nh = new ros::NodeHandle();

    //sub_pose= nh->subscribe("ur5/ee_pose", 1000, &haptic_guidance::robot_pose_callback, this);
    //sub_obj_pose= nh->subscribe("poses_boxes", 1000, &haptic_guidance::object_pose_callback, this);
    //sub_hap_pedal=nh->subscribe ( "ur5/hap_pedal",2,&haptic_guidance::hap_pedal_cb,this );

}
void haptic_guidance::init() {
    std::tie(X, Y, Z) = meshgrid(cv::Range(range[0], range[1]), cv::Range(range[2], range[3]), cv::Range(range[4], range[5]));

    for (int i = range[0]; i <= range[1]; i++) xi.push_back(i);
    for (int i = range[2]; i <= range[3]; i++) yi.push_back(i);
    for (int i = range[4]; i <= range[5]; i++) zi.push_back(i);
/*
    for (int i = range[0]; i <= range[1]; i++){
        double j = i / 2.0; xi.push_back(j);
    }
    for (int i = range[2]; i <= range[3]; i++){
        double j = i / 2.0; yi.push_back(j);
    }
    for (int i = range[4]; i <= range[5]; i++){
        double j = i / 2.0; zi.push_back(j);
    }
*/
    goal[0]=0.124*100;
    goal[1]=0.614*100;
    goal[2]=0.0651*100;

    /*
    goal[0]=obj_pose.pose.position.x*100;
    goal[1]=obj_pose.pose.position.y*100;
    goal[2]=obj_pose.pose.position.z*100;
    */

    gx_itr = std::find(xi.begin(), xi.end(), goal[0]);
    gy_itr = std::find(yi.begin(), yi.end(), goal[1]);
    gz_itr = std::find(zi.begin(), zi.end(), goal[2]);

    gx_idx = std::distance( xi.begin(), gx_itr);
    gy_idx = std::distance( yi.begin(), gy_itr);
    gz_idx = std::distance( zi.begin(), gz_itr);


    std::vector<cv::Mat> f = potential_field(X, Y, Z, goal);

    std::tie(Gx, Gy, Gz) = gradient(f);
}

std::tuple<double,double,double> haptic_guidance::guidance_loop(geometry_msgs::PoseStamped ee_pose) {
    int x,y,z;


     x=ee_pose.pose.position.x*100;
     y=ee_pose.pose.position.y*100;
     z=ee_pose.pose.position.z*100;

    /*
    x=-0.0214*1000;
    y=0.063*1000;
    z=0.0572*1000;
    */
    x_itr = std::find(xi.begin(), xi.end(), x);
    y_itr = std::find(yi.begin(), yi.end(), y);
    z_itr = std::find(zi.begin(), zi.end(), z);

    x_idx = std::distance( xi.begin(), x_itr);
    y_idx = std::distance( yi.begin(), y_itr);
    z_idx = std::distance( zi.begin(), z_itr);


    //if(pedal_on){

        //std::cerr << Gx[z_idx].at<int>(y_idx,x_idx)/8 << std::endl;
        //std::cerr << Gy[z_idx].at<int>(y_idx,x_idx)/8 << std::endl;
        //std::cerr << Gz[z_idx].at<int>(y_idx,x_idx)/8 << std::endl;

        fx = (double)Gx[z_idx].at<int>(y_idx,x_idx)/20;
        fy = (double)Gy[z_idx].at<int>(y_idx,x_idx)/20;
        fz = (double)Gz[z_idx].at<int>(y_idx,x_idx)/20;

    //std::cerr << fx << std::endl;
    //std::cerr << fy << std::endl;
    //std::cerr << fz << std::endl;

    return std::tie(fx,fy,fz);
        //SetHaptic(fx,fy,fz);

    //}

}

std::tuple<std::vector<cv::Mat>, std::vector<cv::Mat>, std::vector<cv::Mat>> haptic_guidance::meshgrid(const cv::Range &xgv, const cv::Range &ygv, const cv::Range &zgv)
{
    int dims[] = { xgv.end, ygv.end, zgv.end };
    std::vector<cv::Mat> X(dims[2]), Y(dims[2]), Z(dims[2]);

    /*
   cv::Mat1d x,y,z;
   std::vector<double> t_x, t_y, t_z;
   for (int i = xgv.start; i <= xgv.end; i++){
       double j = i / 2.0; t_x.push_back(j);
   }
   for (int i = ygv.start; i <= ygv.end; i++){
       double j = i / 2.0; t_y.push_back(j);
   }
   for (int i = zgv.start; i <= zgv.end; i++){
       double j = i / 2.0; t_z.push_back(j);
   }
   */

    cv::Mat1i x,y,z;
    std::vector<int> t_x, t_y, t_z;
    for (int i = xgv.start; i <= xgv.end; i++) t_x.push_back(i);
    for (int i = ygv.start; i <= ygv.end; i++) t_y.push_back(i);
    for (int i = zgv.start; i <= zgv.end; i++) t_z.push_back(i);
    cv::repeat(cv::Mat(t_x).reshape(1,1), cv::Mat(t_y).total(), 1, x);
    cv::repeat(cv::Mat(t_y).reshape(1,1).t(),1, cv::Mat(t_x).total(),y);

    for (int z = 0; z < dims[2]; ++z)
    {
        X[z]=x;
        Y[z]=y;
    }

    for (int i = 0; i < zgv.size(); i++) {
        Z[i]  = t_z[i]*cv::Mat(ygv.end-ygv.start+1, xgv.end-xgv.start+1, 4, cvScalar(1));
    }

    return std::tie(X, Y, Z);
}

std::vector<cv::Mat> haptic_guidance::potential_field(std::vector<cv::Mat> &X,std::vector<cv::Mat> &Y,std::vector<cv::Mat> &Z, int goal[])
{
    std::vector<cv::Mat> field(X.size());
    std::copy ( X.rbegin(), X.rend(), field.begin() );
    int k=goal[0],h=goal[1],w =goal[2];

    std::transform(X.rbegin(), std::next(X.rbegin(), 1), X.rbegin(), // X = X - goal[0]
                   [&k](cv::Mat n) { return n - k; });

    std::transform(Y.rbegin(), std::next(Y.rbegin(), 1), Y.rbegin(), // Y = Y - goal[1]
                   [&h](const cv::Mat &n) { return n - h; });

    std::transform(Z.rbegin(), std::next(Z.rbegin(), Z.size()), Z.rbegin(), // Z = Z - goal[2]
                   [&w](const cv::Mat &n) { return n - w; });

    X[0]=X[0].mul(X[0]); // X^2
    Y[0]=Y[0].mul(Y[0]); // Y^2
    for (int z = 0; z < Z.size(); ++z) Z[z]=Z[z].mul(Z[z]); // Z^2

    std::transform(X.rbegin(), std::next(X.rbegin(), 1), Y.rbegin(), field.rbegin(), // field=X+Y
                   [](const cv::Mat &a, const cv::Mat &b){ return a + b; });

    std::transform (field.begin(), field.end(), Z.begin(), field.begin(), std::plus<cv::Mat>()); // field=field+Z

    /*  2nd VERSION (no variables overwritten)
    std::vector<cv::Mat> A(X.size()),B(Y.size()),C(Z.size());
    std::vector<cv::Mat> field(X.size());
    cv::Mat f = cv::Mat::zeros(cv::Size(X[0].cols,X[0].rows), 4);
    cv::Mat a = cv::Mat::zeros(cv::Size(X[0].cols,X[0].rows), 4);
    cv::Mat b = cv::Mat::zeros(cv::Size(Y[0].cols,Y[0].rows), 4);
    cv::Mat c = cv::Mat::zeros(cv::Size(Z[0].cols,Z[0].rows), 4);
    for (int z = 0; z < X.size(); ++z) field[z].push_back(f);
    for (int z = 0; z < X.size(); ++z) A[z].push_back(a);
    for (int z = 0; z < X.size(); ++z) B[z].push_back(b);
    for (int z = 0; z < X.size(); ++z) C[z].push_back(c);

    std::vector<int> k,h; int w =goal[2];
    k.push_back(goal[0]); h.push_back(goal[1]);
    cv::Mat kk,hh;
    cv::repeat(cv::Mat(k).reshape(1,1), Y[0].rows,X[0].cols, kk);
    cv::repeat(cv::Mat(h).reshape(1,1), Y[0].rows,X[0].cols, hh);
    std::vector<cv::Mat> K(X.size()); for (int i=0;i<X.size();i++) K[i].push_back(kk);
    std::vector<cv::Mat> H(X.size()); for (int i=0;i<X.size();i++) H[i].push_back(hh);

    std::transform (X.begin(), X.end(), K.begin(), A.begin(), std::minus<cv::Mat>()); // X = X - goal[0]
    std::transform (Y.begin(), Y.end(), H.begin(), B.begin(), std::minus<cv::Mat>()); // Y = Y - goal[1]
    std::transform (Z.rbegin(), std::next(Z.rbegin(), Z.size()), C.rbegin(), [&w](const cv::Mat &n) { return n - w; });  // Z = Z - goal[2]

    for (int z = 0; z < A.size(); ++z) A[z]=A[z].mul(A[z]); // X^2
    for (int z = 0; z < B.size(); ++z) B[z]=B[z].mul(B[z]); // Y^2
    for (int z = 0; z < C.size(); ++z) C[z]=C[z].mul(C[z]); // Z^2

    std::transform (A.begin(), A.end(), B.begin(), field.begin(), std::plus<cv::Mat>()); // field=X+Y
    std::transform (field.begin(), field.end(), C.begin(), field.begin(), std::plus<cv::Mat>()); // field=field+Z

    for (int z = 0; z < A.size(); ++z) std::cerr << A[z] << std::endl;
    for (int z = 0; z < A.size(); ++z) std::cerr << B[z] << std::endl;
    for (int z = 0; z < A.size(); ++z) std::cerr << C[z] << std::endl;
    for (int z = 0; z < A.size(); ++z) std::cerr << field[z] << std::endl;*/
    return field;
}
std::vector<cv::Mat> haptic_guidance::diff_vector(const cv::Mat &n, int m){
    cv::Mat H = n.colRange(2,n.cols)-n.colRange(0,n.cols-2);
    cv::repeat(H.reshape(1,1), m, 1, H);
    H.convertTo(H,CV_32FC1);
    std::vector<cv::Mat> M;
    M.push_back(H.t());
    return M;
}
std::vector<cv::Mat> haptic_guidance::gradient_computation(std::vector<cv::Mat> &f, cv::Mat &h, std::vector<cv::Mat> &H, int &dim){

    std::vector<cv::Mat> v0(f.size()), v1(f.size()), v2(f.size()), v3(f.size()), v4(f.size()), v5(f.size());
    cv::Mat g = cv::Mat::zeros(cv::Size(f[0].cols, f[0].rows), 4);
    std::vector<cv::Mat> G(f.size());
    switch (dim) {
        case 1:
            for (int i = 0; i < f.size(); i++) {
                cv::Mat sub0 = f[i].rowRange(0, 1);
                cv::Mat sub1 = f[i].rowRange(1, 2);
                v0[i].push_back(sub0);
                v1[i].push_back(sub1);
                cv::Mat sub2 = f[i].rowRange(f[0].rows - 2, f[0].rows - 1);
                cv::Mat sub3 = f[i].rowRange(f[0].rows - 1, f[0].rows);
                v2[i].push_back(sub2);
                v3[i].push_back(sub3);
                cv::Mat sub4 = f[i].rowRange(0, f[0].rows - 2);
                cv::Mat sub5 = f[i].rowRange(2, f[0].rows);
                sub4.convertTo(sub4, CV_32FC1);
                sub5.convertTo(sub5, CV_32FC1);
                v4[i].push_back(sub4);
                v5[i].push_back(sub5);
            }
            break;

        case 2:
            for (int i = 0; i < f.size(); i++) {
                cv::Mat sub0 = f[i].colRange(0, 1);
                cv::Mat sub1 = f[i].colRange(1, 2);
                v0[i].push_back(sub0);
                v1[i].push_back(sub1);
                cv::Mat sub2 = f[i].colRange(f[0].cols - 2, f[0].cols - 1);
                cv::Mat sub3 = f[i].colRange(f[0].cols - 1, f[0].cols);
                v2[i].push_back(sub2);
                v3[i].push_back(sub3);
                cv::Mat sub4 = f[i].colRange(0, f[0].cols - 2);
                cv::Mat sub5 = f[i].colRange(2, f[0].cols);
                sub4.convertTo(sub4, CV_32FC1);
                sub5.convertTo(sub5, CV_32FC1);
                v4[i].push_back(sub4);
                v5[i].push_back(sub5);
            }
            break;
        case 3:
            for (int z = 0; z < f.size(); ++z) f[z]=f[z].t();
            std::vector<cv::Mat> nf;
            cv::Mat n = cv::Mat::zeros(cv::Size(1,f[0].rows*f[0].cols), 4);
            for (int z = 0; z < f.size(); ++z) cv::hconcat(n, f[z].reshape(1,f[0].rows*f[0].cols), n);
            n=n.colRange(cv::Range(1,f.size()+1));
            nf.push_back(n);
            v0.resize(nf.size()), v1.resize(nf.size()), v2.resize(nf.size());
            v3.resize(nf.size()), v4.resize(nf.size()), v5.resize(nf.size());
            G.resize(nf.size());
            g=g.reshape(1,f[0].rows*f[0].cols);
            for (int i = 0; i < nf.size(); i++) {
                cv::Mat sub0 = nf[i].colRange(0, 1);
                cv::Mat sub1 = nf[i].colRange(1, 2);
                v0[i].push_back(sub0);
                v1[i].push_back(sub1);
                cv::Mat sub2 = nf[i].colRange(nf[0].cols - 2, nf[0].cols - 1);
                cv::Mat sub3 = nf[i].colRange(nf[0].cols - 1, nf[0].cols);
                v2[i].push_back(sub2);
                v3[i].push_back(sub3);
                cv::Mat sub4 = nf[i].colRange(0, nf[0].cols - 2);
                cv::Mat sub5 = nf[i].colRange(2, nf[0].cols);
                sub4.convertTo(sub4, CV_32FC1);
                sub5.convertTo(sub5, CV_32FC1);
                v4[i].push_back(sub4);
                v5[i].push_back(sub5);
            }
            break;
    }

    for (int z = 0; z < G.size(); ++z) G[z].push_back(g);

    std::transform(v0.begin(), v0.end(), v1.begin(), v1.begin(), std::minus<cv::Mat>()); // 1st row gy (-(f(2,:)-f(1,:)))
    //std::transform(v1.begin(), v1.end(), v0.begin(), v1.begin(), std::minus<cv::Mat>()); // 1st row gy ((f(2,:)-f(1,:)))

    double d = h.at<int>(1) - h.at<int>(0); // h(2)-h(1)

    std::transform(v1.begin(), v1.end(), v1.begin(),
                   [&d](const cv::Mat &c) { return c / d; }); // (-(f(2,:)-f(1,:)))/(h(2)-h(1))

    std::transform(v2.begin(), v2.end(), v3.begin(), v3.begin(),std::minus<cv::Mat>()); // nth row gy (-(f(n,:)-f(n-1,:)))
    //std::transform(v3.begin(), v3.end(), v2.begin(), v3.begin(),std::minus<cv::Mat>()); // nth row gy ((f(n,:)-f(n-1,:)))

    d = h.at<int>(h.cols - 1) - h.at<int>(h.cols - 2); // (h(end)-h(end-1))
    std::transform(v3.begin(), v3.end(), v3.begin(),
                   [&d](const cv::Mat &c) { return c / d; }); // (-(f(n,:)-f(n-1,:)))/(h(end)-h(end-1))

    std::transform(v4.begin(), v4.end(), v5.begin(), v5.begin(), std::minus<cv::Mat>()); // 2nd to (n-1)th row gy (-(f(3:n,:)-f(1:n-2,:)))
    //std::transform(v5.begin(), v5.end(), v4.begin(), v5.begin(), std::minus<cv::Mat>()); // 2nd to (n-1)th row gy( (f(3:n,:)-f(1:n-2,:)))

    std::transform(H.begin(), H.end(), H.begin(), [](const cv::Mat &c) { return 1 / c; });
    for (int z = 0; z < G.size(); ++z) v5[z] = v5[z].mul(H[0]);


    switch (dim) {
        case 1:
            for (int z = 0; z < G.size(); ++z) v1[z].copyTo(G[z].rowRange(0, 1));
            for (int z = 0; z < G.size(); ++z) v3[z].copyTo(G[z].rowRange(f[0].rows - 1, f[0].rows));
            for (int z = 0; z < G.size(); ++z) v5[z].copyTo(G[z].rowRange(1, f[0].rows - 1));
            break;
        case 2:
            for (int z = 0; z < G.size(); ++z) v1[z].copyTo(G[z].colRange(0, 1));
            for (int z = 0; z < G.size(); ++z) v3[z].copyTo(G[z].colRange(f[0].cols - 1, f[0].cols));
            for (int z = 0; z < G.size(); ++z) v5[z].copyTo(G[z].colRange(1, f[0].cols - 1));
            break;
        case 3:

            G.resize(f.size()); for (int z = 1; z < G.size(); ++z) G[z].push_back(g);
            g=g.reshape(f[0].rows,f[0].cols);
            for (int z = 0; z < G.size(); ++z) G[z]=G[z].reshape(f[0].rows,f[0].cols);

            G[0].push_back(v1[0].reshape(f[0].rows,f[0].cols)); G[0]=G[0].rowRange(f[0].cols,2*f[0].cols); // 1st matrix
            G[G.size()-1].push_back(v3[0].reshape(f[0].rows,f[0].cols)); G[G.size()-1]=G[G.size()-1].rowRange(f[0].cols,2*f[0].cols); // Last matrix
            for (int z = 0; z < v5[0].cols; ++z) { // Matrices in the middle
                cv::Mat aux= v5[0].col(z);
                aux.convertTo(aux, 4);
                aux = aux.reshape(f[0].rows, f[0].cols);
                G[z+1].push_back(aux);
                G[z+1] = G[z+1].rowRange(f[0].cols, 2*f[0].cols);
            }

            break;
    }
    return G;
}

std::tuple<std::vector<cv::Mat>, std::vector<cv::Mat>, std::vector<cv::Mat>> haptic_guidance::gradient(std::vector<cv::Mat> &f) {

    int dim;
    std::vector<int> x,y,z;
    cv::Mat hx, hy, hz;

    for (int i = 1; i <= f[0].rows; i++) y.push_back(i);
    cv::repeat(cv::Mat(y).reshape(1,1), 1,1, hy);

    for (int i = 1; i <= f[0].cols; i++) x.push_back(i);
    cv::repeat(cv::Mat(x).reshape(1,1), 1,1, hx);

    for (int i = 1; i <= f.size(); i++) z.push_back(i);
    cv::repeat(cv::Mat(z).reshape(1,1), 1,1, hz);

    std::vector<cv::Mat> Hy = diff_vector(hy,f[0].cols);
    std::vector<cv::Mat> Hx = diff_vector(hx,f[0].rows);
    std::vector<cv::Mat> Hz = diff_vector(hz,f[0].rows*f[0].cols);
    Hx[0] = Hx[0].t();
    Hz[0] = Hz[0].t();

    std::vector<cv::Mat> Gy = gradient_computation(f, hy, Hy, dim=1);

    std::vector<cv::Mat> Gx = gradient_computation(f, hx, Hx, dim=2);

    std::vector<cv::Mat> Gz = gradient_computation(f, hz, Hz, dim=3);

    return std::tie(Gx,Gy,Gz);
}
bool haptic_guidance::SetHaptic(double &x, double &y, double &z){

    dhdEnableForce(DHD_ON);
    if(pedal_on){

        dhdSetForceAndTorqueAndGripperForce( x, y, z, 0, 0, 0, 0);

    }
    else{
       dhdSetForceAndTorqueAndGripperForce(0.0,0.0,0.0,0.0,0.0,0.0,0.0);
    }
}

void haptic_guidance::hap_pedal_cb ( const std_msgs::Bool::ConstPtr &msg ) {
    /*if ( msg->data ) {
        if ( !pedal_on ) {
            ROS_WARN("Pedal pressed");
            ee_pose=cur_ee_pose;
        }
    }*/
    pedal_on=msg->data;
    //ROS_INFO_STREAM("" << pedal_on);
}

void haptic_guidance::robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    ee_pose=*msg;
    //ROS_INFO_STREAM("" << cur_ee_pose);
}
/*
void haptic_guidance::object_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    obj_pose=*msg;
}*/
/*
int main(int argc, char** argv) {
    ros::init(argc, argv, "haptic_guidance");
    haptic_guidance *hap_gui=new haptic_guidance();
    ros::Rate rate(100);
    hap_gui->init();
    //    drdStop();
    //dhdSetGravityCompensation(DHD_ON);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        hap_gui->guidance_loop();
    }
    delete hap_gui;
    return 0;

}
 */