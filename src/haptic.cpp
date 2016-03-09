#include <soma_ur5/haptic.h>


Haptic::Haptic(){
    this->nh=new ros::NodeHandle();
    initialize_haptic();

    pub_hap_pose=nh->advertise<geometry_msgs::Pose>("haptic_pose",5);
    pub_robot_com=nh->advertise<geometry_msgs::Pose>("/goal_pose",5);
    sub_pose= nh->subscribe("ee_pose", 1000, &Haptic::robot_pose_callback, this);
    nh->getParam("scale_factor", scale_factor);

    goto_initial();

}
Haptic::~Haptic(){
    dhdClose ();
}

geometry_msgs::Pose Haptic::scale_pose(geometry_msgs::Pose in,std::string mode){
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

    //out.orientation=in.orientation;//FIXME
    out.orientation.w=1.0;
    return out;
}


bool Haptic::goto_initial(){
    while(ee_pose.orientation.x==0 && ee_pose.orientation.y==0 && ee_pose.orientation.z==0&& ee_pose.orientation.w==0){
        ros::spinOnce();
        ros::Duration(0.2).sleep();
        printf("waiting for robot position...\n");
    }
    geometry_msgs::Pose i_p;
    i_p=scale_pose(ee_pose,"r2h");
    move_haptic(i_p);
    return true;
}

bool Haptic::move_haptic(geometry_msgs::Pose in){

    double com[7];
    tf::Quaternion quat;
    tf::quaternionMsgToTF(in.orientation,quat);
    // tf::Matrix3x3(quat).getEulerZYX(com[3],com[4],com[5]); //FIXME
    com[0]=in.position.x;
    com[1]=in.position.y;
    com[2]=in.position.z;
    com[6]=0;
    ROS_INFO("pos: %.2f %.2f %.2f %.2f %.2f %.2f %2f\n",com[0],com[1],com[2],com[3],com[4],com[5],com[6]);
    drdMoveTo(com,true);

    return true;
}

void Haptic::robot_pose_callback(const geometry_msgs::Pose::ConstPtr &msg){
    ee_pose=*msg;
}

int Haptic::initialize_haptic(){
    int major, minor, release, revision;
    dhdGetSDKVersion (&major, &minor, &release, &revision);
    printf ("Force Dimension - Robot Control Example %d.%d.%d.%d\n", major, minor, release, revision);
    printf ("(C) 2001-2015 Force Dimension\n");
    printf ("All Rights Reserved.\n\n");

    dhdEnableExpertMode ();

    // open the first available device
    if (drdOpen () < 0) {
        printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);
        return -1;
    }
    printf ("%s device detected\n\n", dhdGetSystemName());


    // print out device identifier
    if (!drdIsSupported()) {
        printf ("unsupported device\n");
        printf ("exiting...\n");
        dhdSleep (2.0);
        drdClose ();
        return -1;
    }
    // initialize if necessary
    if (!drdIsInitialized() && (drdAutoInit() < 0)) {
        printf ("error: initialization failed (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);
        return -1;
    }
    /*
    // start robot control loop
    if (drdStart() < 0) {
        printf ("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);
        return -1;
    }
*/


        dhdEnableForce(DHD_ON);


}

bool Haptic::GetHapticInfo(geometry_msgs::Pose &h_pose){
    //double x,y,z;
    double ori[3][3];
    tf::Quaternion q;
    dhdGetPosition(&(h_pose.position.x),&(h_pose.position.y),&(h_pose.position.z));
    dhdGetOrientationFrame(ori);
    tf::Matrix3x3(ori[0][0],ori[0][1],ori[0][2],
            ori[1][0],ori[1][1],ori[1][2],
            ori[2][0],ori[2][1],ori[2][2]).getRotation(q);

    tf::quaternionTFToMsg(q,h_pose.orientation);
    hap_pose=h_pose;
    pub_hap_pose.publish(h_pose);
    return true;
}


bool Haptic::SetHaptic(){
    // double pos[7]={0,0,0,0,0,0,0};
    //    drdMoveTo(pos);
    dhdEnableForce(DHD_ON);
    geometry_msgs::Pose see_pose=scale_pose(ee_pose,"r2h");
    ROS_INFO("%.5f %.5f %.5f",see_pose.position.x-hap_pose.position.x,
             see_pose.position.y-hap_pose.position.y,
             see_pose.position.z-hap_pose.position.z+1.0);
    dhdSetForceAndTorqueAndGripperForce (-20*(hap_pose.position.x-see_pose.position.x),
                                         -20*(hap_pose.position.y-see_pose.position.y),
                                         -20*(hap_pose.position.z-see_pose.position.z)+1.0,
                                         0.0, 0.0, 0.0, 0.0);
 //   ROS_INFO("Setting FOrce");

}

bool Haptic::haptic_loop(){
    geometry_msgs::Pose h_pose,com_pose;
    GetHapticInfo(h_pose);
    SetHaptic();
    com_pose=scale_pose(h_pose,"h2r");
    com_pose.orientation=ee_pose.orientation;
    pub_robot_com.publish(com_pose);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "haptic_soma");
    Haptic *hap=new Haptic();
    ros::Rate rate(100);

//    drdStop();
    //dhdSetGravityCompensation(DHD_ON);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        hap->haptic_loop();
    }
    delete hap;
    return 0;

}
