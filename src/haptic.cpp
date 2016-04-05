#include <soma_ur5/haptic.h>


Haptic::Haptic(){
    this->nh=new ros::NodeHandle();
    initialize_haptic();

    pub_hap_pose=nh->advertise<geometry_msgs::PoseStamped>("haptic_pose",5);
    pub_robot_com=nh->advertise<geometry_msgs::PoseStamped>("/goal_pose",5);
    pub_grip=nh->advertise<std_msgs::Float64>("grip_cmd",5);
    pub_pedal=nh->advertise<std_msgs::Bool>("hap_pedal",5);


    sub_pose= nh->subscribe("ee_pose", 1000, &Haptic::robot_pose_callback, this);
    sub_grip= nh->subscribe("grip_feedback", 1000, &Haptic::grip_callback, this);
    sub_force= nh->subscribe("ee_force", 1000, &Haptic::ee_force_callback, this);

    //nh->getParam("scale_factor", scale_factor);

    pedal_on=false;
 //   goto_initial();
    dynamic_reconfigure::Server<soma_ur5::dyn_ur5_hapticConfig>::CallbackType f;
    f=boost::bind(&Haptic::config_cb, this, _1, _2);
    config_server.setCallback(f);


}
Haptic::~Haptic(){
    dhdClose ();
}

void Haptic::config_cb(soma_ur5::dyn_ur5_hapticConfig &config, uint32_t level) {
    ROS_DEBUG("Reconfigure Request.");
        scale_factor=config.scale_factor;
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

    out.orientation=in.orientation;

    return out;
}


bool Haptic::goto_initial(){
    while(ee_pose.pose.orientation.x==0 &&
          ee_pose.pose.orientation.y==0 &&
          ee_pose.pose.orientation.z==0 &&
          ee_pose.pose.orientation.w==0){
        ros::spinOnce();
        ros::Duration(0.2).sleep();
        printf("waiting for robot position...\n");
    }
    geometry_msgs::Pose i_p;
    i_p=scale_pose(ee_pose.pose,"r2h");
    move_haptic(i_p);
    return true;
}

bool Haptic::move_haptic(geometry_msgs::Pose in){

    double com[7];
    tf2::Quaternion quat;
    //tf::quaternionMsgToTF(in.orientation,quat);
    tf2::convert(in.orientation,quat);
    // tf::Matrix3x3(quat).getEulerZYX(com[3],com[4],com[5]); //FIXME
    com[0]=in.position.x;
    com[1]=in.position.y;
    com[2]=in.position.z;
    com[6]=0;
    ROS_INFO("pos: %.2f %.2f %.2f %.2f %.2f %.2f %2f\n",com[0],com[1],com[2],com[3],com[4],com[5],com[6]);
    //  drdMoveTo(com,true);

    return true;
}

void Haptic::robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    ee_pose=*msg;
}

void Haptic::grip_callback(const std_msgs::Float64::ConstPtr &msg){
    grip_val=msg->data;
}

void Haptic::ee_force_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg){
    cur_ee_force=*msg;
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
        ROS_ERROR("Can't open device, try sudo chown $USER /dev/bus/usb/[bus]/[dev]");
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

    /*  // start robot control loop
    if (drdStart() < 0) {
        printf ("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);
        return -1;
    }
    */
    // dhdEnableForce(DHD_ON);

    ROS_INFO("Haptic device initialized");
    return 0;

}

bool Haptic::GetHapticInfo(geometry_msgs::Pose &h_pose){
    //double x,y,z;
    double ori[3][3];
    tf2::Quaternion q;
    dhdGetPosition(&(h_pose.position.x),&(h_pose.position.y),&(h_pose.position.z));
    dhdGetOrientationFrame(ori);

    tf2::Matrix3x3(ori[0][0],ori[0][1],ori[0][2],
            ori[1][0],ori[1][1],ori[1][2],
            ori[2][0],ori[2][1],ori[2][2]).getRotation(q);

    //tf::quaternionTFToMsg(q,h_pose.orientation);
    tf2::convert(q,h_pose.orientation);
    hap_pose.pose=h_pose;
    hap_pose.header.stamp=ros::Time::now();
    hap_pose.header.frame_id="base_link";
    pub_hap_pose.publish(hap_pose);
    std_msgs::Float64 grip_pos;
    double tmp[1];
    dhdGetGripperAngleDeg(tmp);
    grip_pos.data=tmp[0];
    pub_grip.publish(grip_pos);
    if(dhdGetButton(1)==1){
        if(pedal_on==false){
            hap_pose_initial=hap_pose;
            ee_pose_initial=ee_pose;
        }
        pedal_on=true;
    }
    else{
        pedal_on=false;
    }
    std_msgs::Bool pedal_msg;
    pedal_msg.data=pedal_on;
    pub_pedal.publish(pedal_msg);
    return true;
}


bool Haptic::SetHaptic(){
    // double pos[7]={0,0,0,0,0,0,0};
    //    drdMoveTo(pos);
    geometry_msgs::Pose see_pose=scale_pose(ee_pose.pose,"r2h");
    // ROS_INFO("%.5f %.5f %.5f",see_pose.position.x-hap_pose.pose.position.x,
    //          see_pose.position.y-hap_pose.pose.position.y,
    //          see_pose.position.z-hap_pose.pose.position.z);
    dhdEnableForce(DHD_ON);
    if(pedal_on){
    dhdSetForceAndTorqueAndGripperForce(cur_ee_force.wrench.force.x,
                                        cur_ee_force.wrench.force.y,
                                        cur_ee_force.wrench.force.z,
                                        cur_ee_force.wrench.torque.x,
                                        cur_ee_force.wrench.torque.y,
                                        cur_ee_force.wrench.torque.z,
                                        grip_val);
    }
    else{
        dhdSetForceAndTorqueAndGripperForce(0.0,0.0,0.0,0.0,0.0,0.0,0.0);

    }

    // dhdSetGravityCompensation();
    // dhdSetForceAndTorqueAndGripperForce (-100*(hap_pose.pose.position.x-see_pose.position.x),
    //                                      -100*(hap_pose.pose.position.y-see_pose.position.y),
    //                                     -100*(hap_pose.pose.position.z-see_pose.position.z)+1.8,
    //                                     0.0, 0.0, 0.0, -0.1);
}
geometry_msgs::Pose Haptic::diff_pose(geometry_msgs::Pose in){
    tf2::Quaternion q1,q2;
    geometry_msgs::Pose tmp;

   // tf::quaternionMsgToTF(hap_pose_initial.pose.orientation,q1);
   // tf::quaternionMsgToTF(in.orientation,q2);
   // tf::quaternionTFToMsg(q2*q1.inverse(),tmp.orientation);

    tf2::convert(hap_pose_initial.pose.orientation,q1);
    tf2::convert(in.orientation,q2);
    tf2::convert(q2*q1.inverse(),tmp.orientation);

    tmp.position.x=in.position.x-hap_pose_initial.pose.position.x;
    tmp.position.y=in.position.y-hap_pose_initial.pose.position.y;
    tmp.position.z=in.position.z-hap_pose_initial.pose.position.z;
    return tmp;
}

bool Haptic::haptic_loop(){
    geometry_msgs::PoseStamped h_pose,com_pose;
    geometry_msgs::Pose d_pose;
    GetHapticInfo(h_pose.pose);
    SetHaptic();
    if(pedal_on){
        ///R1(rot)=R0*(H0'*H1)
        d_pose=scale_pose(diff_pose(h_pose.pose),"h2r");
        //tf::quaternionTFToMsg(
        //            (utils::Pose2Transform(d_pose)*utils::Pose2Transform(ee_pose_initial.pose)).getRotation(),
        //            com_pose.pose.orientation);
        tf2::convert( (utils::Pose2Transform(d_pose)*utils::Pose2Transform(ee_pose_initial.pose)).getRotation(),
                       com_pose.pose.orientation);


        ///R1(t)=R0+(H0'*H1)
        com_pose.pose.position.x=ee_pose_initial.pose.position.x+d_pose.position.x;
        com_pose.pose.position.y=ee_pose_initial.pose.position.y+d_pose.position.y;
        com_pose.pose.position.z=ee_pose_initial.pose.position.z+d_pose.position.z;

        com_pose.header.stamp=ros::Time::now();
        com_pose.header.frame_id="base_link";
        pub_robot_com.publish(com_pose);


        //ROS_INFO("d_pose: %.3f %.3f %.3f",d_pose.position.x,d_pose.position.y,d_pose.position.z);
        double Td[16];
        utils::pose2array(d_pose,Td);
        ROS_INFO_STREAM(utils::print_matrix(4,4,Td,"d_pose"));

        ROS_INFO("com_pose: %.3f %.3f %.3f",com_pose.pose.position.x,com_pose.pose.position.y,com_pose.pose.position.z);
    }
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
