#include <soma_ur5/haptic.h>
#include "netft_rdt_driver/String_cmd.h"
#include "soma_ur5/haptic_guidance.h"


Haptic::Haptic(haptic_guidance &gui){
//Haptic::Haptic(){
    this->nh=new ros::NodeHandle();
    initialize_haptic();
    //this->gui=new haptic_guidance;

    gui.init();
    pub_hap_pose=nh->advertise<geometry_msgs::PoseStamped>("haptic_pose",5);
    pub_d_pose=nh->advertise<geometry_msgs::PoseStamped>("d_pose",5);
    pub_robot_com=nh->advertise<geometry_msgs::PoseStamped>("goal_pose",5);
    dbg_robot_com=nh->advertise<geometry_msgs::PoseStamped>("dbg_pose",5);
    pub_grip=nh->advertise<std_msgs::Float32>("cmd_gripper",5); //grip_cmd
    dbg_pub_grip=nh->advertise<scoop_msgs::Scoop>("dbg_cmd_gripper",5);
    pub_pedal=nh->advertise<std_msgs::Bool>("hap_pedal",5);
    ft_client = nh->serviceClient<netft_rdt_driver::String_cmd>("bias_cmd");

    sub_pose= nh->subscribe("ee_pose", 1000, &Haptic::robot_pose_callback, this);
    sub_grip= nh->subscribe("grip_feedback", 1000, &Haptic::grip_callback, this); 
    sub_force= nh->subscribe("ee_force", 1000, &Haptic::ee_force_callback, this);

    sub_force_bl= nh->subscribe("ee_force_blink", 1000, &Haptic::ee_force_bl_callback, this);

    sub_mapping= nh->subscribe("mapping", 10, &Haptic::mapping_callback, this);

    //nh->getParam("scale_factor", scale_factor);

    pedal_on=false;
 //   goto_initial();
    dynamic_reconfigure::Server<soma_ur5::dyn_ur5_hapticConfig>::CallbackType f;
    f=boost::bind(&Haptic::config_cb, this, _1, _2);
    config_server.setCallback(f);

    // CAMERA ORIENTATION (MAPPING 4) - rosrun tf tf_echo world kinect2_link 
    // CHECK THESE VALUES!!!!
    
    camera_pose.pose.orientation.x=-0.583;
    camera_pose.pose.orientation.y=0.627;
    camera_pose.pose.orientation.z=-0.371;
    camera_pose.pose.orientation.w=0.36;

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
    if(pedal_on==true) {
        dbg_robot_com.publish(ee_pose);
    }
}

void Haptic::grip_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg){ //std_msgs::Float64::ConstPtr
    //grip_val=msg->data; 
    paletta_torque =*msg;
    //grip_val = paletta_torque.wrench.torque.x;

}

void Haptic::ee_force_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg){
    cur_ee_force=*msg;
}

void Haptic::ee_force_bl_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg){
    cur_ee_force_bl=*msg;
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
    dhdEnableExpertMode ();
    
    dhdEnableForce(DHD_ON);
    dhdSetForceAndTorqueAndGripperForce(0.0,0.0,0.0,0.0,0.0,0.0,0.0);


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
    std_msgs::Float32 grip_pos;
    std_msgs::String cmd;
    double tmp[1];
    dhdGetGripperAngleDeg(tmp);
    grip_pos.data=tmp[0];
        //grip_pos.data=840+(tmp[0]/29.534)*(2492-840); // Mapping Gripper->Paletta's Fingers
    //grip_pos.data=540+(tmp[0]/29.534)*(2492-540); // Mapping Gripper->Paletta's Fingers (Closed Gripper- Closed Hand)
    grip_pos.data=540+((29.534-tmp[0])/29.534)*(2492-540); // Mapping Gripper->Paletta's Fingers (Opened Gripper- Closed Hand)
    //std_msgs::Float32 prev=grip_pos;
    fake_gripper.data=grip_pos.data;
    fake_gripper.stamp = ee_pose.header.stamp;
    /*
    if (grip_pos.data < 1){
        cmd.data = "o";
        pub_grip.publish(cmd);
    }
    else if (grip_pos.data > 28){
        cmd.data = "c";
        pub_grip.publish(cmd);
    }*/
    if(dhdGetButton(1)==1){
        if(pedal_on==false){
            hap_pose_initial=hap_pose;
            ee_pose_initial=ee_pose;
            //ros::Time b=ros::Time::now();
        }
        pedal_on=true;
        aux=ros::Time::now();
    }
    else{
        pedal_on=false;
        last_command=aux;
    }
    std_msgs::Bool pedal_msg;
    pedal_msg.data=pedal_on;
    pub_pedal.publish(pedal_msg);

    if (pedal_on==true) {
        //ros::Time a=ros::Time::now();
        //last_command=ros::Time::now();
        //last_command=a;

        //double t=(aux-last_command).toSec(); //Doppio-tap su pedale
        //if (t < 0.5){
        pub_grip.publish(grip_pos);
        //prev=grip_pos;
        dbg_pub_grip.publish(fake_gripper);
        //}
    }else {
        dbg_pub_grip.publish(fake_gripper);
    }
    
    return true;
}


bool Haptic::SetHaptic(int &mapping, haptic_guidance &gui, int &hap_gui){
//bool Haptic::SetHaptic(int &mapping){
    // double pos[7]={0,0,0,0,0,0,0};
    //    drdMoveTo(pos);
    geometry_msgs::Pose see_pose=scale_pose(ee_pose.pose,"r2h");
    // ROS_INFO("%.5f %.5f %.5f",see_pose.position.x-hap_pose.pose.position.x,
    //          see_pose.position.y-hap_pose.pose.position.y,
    //          see_pose.position.z-hap_pose.pose.position.z);
    //double fx, fy, fz;
    //std::tie(fx,fy, fz)=gui.guidance_loop();
    nh->getParam("enable_guidance",hap_gui);
    if (hap_gui==1) {
        std::tie(fx, fy, fz) = gui.guidance_loop(ee_pose);
        std::cerr << "Guidance Enabled"<< std::endl;
    }else{
        fx=0;fy=0;fz=0;
    }
    //std::cerr << hap_gui << std::endl;
    //std::cerr << fx << std::endl;
    //std::cerr << fy << std::endl;
    //std::cerr << fz << std::endl;
    dhdEnableForce(DHD_ON);
    if(pedal_on){
        switch(mapping){
            case 1: 
        dhdSetForceAndTorqueAndGripperForce(
                                        //PALETTA FRAME
                                        -cur_ee_force.wrench.force.z,
                                        cur_ee_force.wrench.force.x,
                                        -cur_ee_force.wrench.force.y,
                                        -cur_ee_force.wrench.torque.z,
                                        cur_ee_force.wrench.torque.x,
                                        -cur_ee_force.wrench.torque.y,
                                        grip_val);
                break;
            case 2:
          dhdSetForceAndTorqueAndGripperForce(
                                        //PALETTA FRAME
                                        -cur_ee_force.wrench.force.z,
                                        -cur_ee_force.wrench.force.y,
                                        -cur_ee_force.wrench.force.x,
                                        -cur_ee_force.wrench.torque.z,
                                        -cur_ee_force.wrench.torque.y,
                                        -cur_ee_force.wrench.torque.x,
                                        grip_val);
                break;
            
            case 3:
            dhdSetForceAndTorqueAndGripperForce(
                                         -cur_ee_force_bl.wrench.force.y/5,
                                         cur_ee_force_bl.wrench.force.x/5,
                                         cur_ee_force_bl.wrench.force.z/5,
                                         -cur_ee_force_bl.wrench.torque.y/5,
                                         cur_ee_force_bl.wrench.torque.x/5,
                                         cur_ee_force_bl.wrench.torque.z/5,
                                        grip_val);
                break;

            case 4:
            dhdSetForceAndTorqueAndGripperForce(
                                         fx+cur_ee_force_bl.wrench.force.x/5,
                                         fy+cur_ee_force_bl.wrench.force.y/5,
                                         fz+cur_ee_force_bl.wrench.force.z/5,
                                         cur_ee_force_bl.wrench.torque.x/5,
                                         cur_ee_force_bl.wrench.torque.y/5,
                                         cur_ee_force_bl.wrench.torque.z/5,
                                        grip_val);
                /*dhdSetForceAndTorqueAndGripperForce(
                        fx,
                        fy,
                        fz,
                        0,
                        0,
                        0,
                        grip_val);*/
            //ADD Haptic Guidance Values (fx fy fz)
                break;

            case 5: //TO BE VERIFIED
            dhdSetForceAndTorqueAndGripperForce(
                                         -cur_ee_force_bl.wrench.force.x/5,
                                         -cur_ee_force_bl.wrench.force.y/5,
                                         cur_ee_force_bl.wrench.force.z/5,
                                         -cur_ee_force_bl.wrench.torque.x/5,
                                         -cur_ee_force_bl.wrench.torque.y/5,
                                         cur_ee_force_bl.wrench.torque.z/5,
                                        grip_val);
                break;

            case 6:
            dhdSetForceAndTorqueAndGripperForce( 
                                        cur_ee_force.wrench.force.y,
                                        -cur_ee_force.wrench.force.x,
                                        -cur_ee_force.wrench.force.z,
                                        cur_ee_force.wrench.torque.y,
                                        -cur_ee_force.wrench.torque.x,
                                        -cur_ee_force.wrench.torque.z,
                                        grip_val);
                break;
        }
    // dhdSetForceAndTorqueAndGripperForce(cur_ee_force.wrench.force.x,
    //                                     cur_ee_force.wrench.force.y,
    //                                     cur_ee_force.wrench.force.z,
    //                                     cur_ee_force.wrench.torque.x,
    //                                     cur_ee_force.wrench.torque.y,
    //                                     cur_ee_force.wrench.torque.z,
    //                                     grip_val);
    cur_ee_force=geometry_msgs::WrenchStamped();
  //  grip_val=0;
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
geometry_msgs::Pose Haptic::diff_pose(geometry_msgs::Pose in, int &mapping){
    tf2::Quaternion q1,q2;
    geometry_msgs::Pose tmp;

   // tf::quaternionMsgToTF(hap_pose_initial.pose.orientation,q1);
   // tf::quaternionMsgToTF(in.orientation,q2);
   // tf::quaternionTFToMsg(q2*q1.inverse(),tmp.orientation);

    tf2::convert(hap_pose_initial.pose.orientation,q1);
    tf2::convert(in.orientation,q2);
    tf2::convert(q2*q1.inverse(),tmp.orientation);
    
    switch(mapping){
            case 1:  // PALETTA FRAME (First-person View)
            tmp.position.x=in.position.y-hap_pose_initial.pose.position.y;
            tmp.position.y=-(in.position.z-hap_pose_initial.pose.position.z);
            tmp.position.z=-(in.position.x-hap_pose_initial.pose.position.x);
            break;

            case 2: // PALETTA FRAME (First-person View - 90° rot)
            tmp.position.x=(in.position.z-hap_pose_initial.pose.position.z);
            tmp.position.y=(in.position.y-hap_pose_initial.pose.position.y);
            tmp.position.z=-(in.position.x-hap_pose_initial.pose.position.x);
            break;

            case 3: // FIXED FRAME - Base Link (Camera View)
            tmp.position.x=in.position.y-hap_pose_initial.pose.position.y;
            tmp.position.y=-(in.position.x-hap_pose_initial.pose.position.x);
            tmp.position.z=in.position.z-hap_pose_initial.pose.position.z;
            break;

            case 4: // FIXED FRAME - Kinect2 Link
            tmp.position.x=(in.position.y-hap_pose_initial.pose.position.y);
            tmp.position.y=-(in.position.z-hap_pose_initial.pose.position.z);
            tmp.position.z=-(in.position.x-hap_pose_initial.pose.position.x);
            break;

            case 5:  // FIXED FRAME - Base Link (Mirrored)
            tmp.position.x=-(in.position.x-hap_pose_initial.pose.position.x);
            tmp.position.y=-(in.position.y-hap_pose_initial.pose.position.y);
            tmp.position.z=in.position.z-hap_pose_initial.pose.position.z;
            break;

            case 6:  // PALETTA FRAME - (First-person View - Mirrored)
            tmp.position.x=-(in.position.x-hap_pose_initial.pose.position.x);
            tmp.position.y=-(in.position.z-hap_pose_initial.pose.position.z);
            tmp.position.z=-(in.position.y-hap_pose_initial.pose.position.y);
            break;
        }
    return tmp;
}

void Haptic::bias_sensor(){
    netft_rdt_driver::String_cmd srv;
    srv.request.cmd  = "bias";
    srv.response.res = "";
    if (ft_client.call(srv))
    {
      ROS_INFO_STREAM("net_ft res: " << srv.response.res);
    }else{
      ROS_ERROR("Failed to call netft bias service");
    }
}

/* ROSSERVICE
bool change_mapping(soma_ur5::ch_mapping::Request &req, soma_ur5::ch_mapping::Response &res){
    //geometry_msgs::Pose new_pose;
    res.map_out = req.map_in;
    //mapping = res.map_out;
    //ROS_INFO("Mapping has been changed: [%d]", mapping);
    return true;
    }
*/
void Haptic::mapping_callback(const std_msgs::Int32::ConstPtr& msg){
    map=*msg;
}

bool Haptic::haptic_loop(haptic_guidance &gui){
//bool Haptic::haptic_loop(){
    geometry_msgs::PoseStamped h_pose,com_pose, dd_pose;
    geometry_msgs::Pose d_pose, tmp_pose;
    //std::tie(fx,fy,fz)=gui.guidance_loop();
    //std::cerr << fx << std::endl;
    GetHapticInfo(h_pose.pose);
    SetHaptic(mapping, gui, hap_gui);
    //SetHaptic(mapping);
    if(pedal_on){
        bias_sensor();
        ///R1(rot)=R0*(H0'*H1)
        
        ///R1(t)=R0+(H0'*H1)

         if (map.data != 0){
            mapping = map.data;    
         }

        d_pose=scale_pose(diff_pose(h_pose.pose, mapping),"h2r");
         
        //camera_pose.pose.position = ee_pose.pose.position;
        
                //tf2::convert((utils::Pose2Transform(d_pose)).getRotation(),
                //       dd_pose.pose.orientation);
        
        switch(mapping){
            case 1: // PALETTA FRAME
                tf2::convert((utils::Pose2Transform(d_pose)*utils::Pose2Transform(ee_pose_initial.pose)).getRotation(),
                       com_pose.pose.orientation);
                tmp_pose=d_pose; 
                d_pose.orientation.x=tmp_pose.orientation.y;
                d_pose.orientation.y=-tmp_pose.orientation.z;
                d_pose.orientation.z=-tmp_pose.orientation.x;

                com_pose.pose.position.x=ee_pose_initial.pose.position.x+d_pose.position.x;
                com_pose.pose.position.y=ee_pose_initial.pose.position.y+d_pose.position.y;
                com_pose.pose.position.z=ee_pose_initial.pose.position.z+d_pose.position.z;

                com_pose.pose=utils::Transform2Pose(utils::Pose2Transform(ee_pose_initial.pose)*utils::Pose2Transform(d_pose)); //NEW
        
                break;

            case 2: // PALETTA FRAME - 90 DEG ROT
                tmp_pose=d_pose;
                d_pose.orientation.x=tmp_pose.orientation.z;
                d_pose.orientation.y=tmp_pose.orientation.y;
                d_pose.orientation.z=-tmp_pose.orientation.x;
                com_pose.pose.position.x =ee_pose_initial.pose.position.x + d_pose.position.x;
                com_pose.pose.position.y =ee_pose_initial.pose.position.y + d_pose.position.y;
                com_pose.pose.position.z =ee_pose_initial.pose.position.z + d_pose.position.z;
                //tf2::convert(utils::Pose2Transform(d_pose)*(utils::Pose2Transform(ee_pose_initial.pose)).getRotation(),com_pose.pose.orientation);
                com_pose.pose=utils::Transform2Pose(utils::Pose2Transform(ee_pose_initial.pose)*utils::Pose2Transform(d_pose)); //NEW
                break;
            
            case 3:
                tmp_pose=d_pose; // FIXED FRAME (BASE_LINK)
                d_pose.orientation.x=tmp_pose.orientation.y;
                d_pose.orientation.y=-tmp_pose.orientation.x;
                //d_pose.orientation.z=-tmp_pose.orientation.x;
                dd_pose.pose=utils::Transform2Pose(utils::Pose2Transform(d_pose)); //NEW
                com_pose.pose.position.x =ee_pose_initial.pose.position.x + dd_pose.pose.position.x;
                com_pose.pose.position.y =ee_pose_initial.pose.position.y + dd_pose.pose.position.y;
                com_pose.pose.position.z =ee_pose_initial.pose.position.z + dd_pose.pose.position.z;
                tf2::convert(utils::Pose2Transform(d_pose)*(utils::Pose2Transform(ee_pose_initial.pose)).getRotation(),com_pose.pose.orientation);

                break;

            case 4:
                tmp_pose=d_pose; // FIXED FRAME (KINECT2_LINK)
                d_pose.orientation.x=-tmp_pose.orientation.x;
                d_pose.orientation.y=-tmp_pose.orientation.y;
                d_pose.orientation.z=tmp_pose.orientation.z;
                dd_pose.pose=utils::Transform2Pose(utils::Pose2Transform(camera_pose.pose)*utils::Pose2Transform(d_pose)); //NEW
                com_pose.pose.position.x =ee_pose_initial.pose.position.x + dd_pose.pose.position.x;
                com_pose.pose.position.y =ee_pose_initial.pose.position.y + dd_pose.pose.position.y;
                com_pose.pose.position.z =ee_pose_initial.pose.position.z + dd_pose.pose.position.z;
                //tf2::convert(utils::Pose2Transform(camera_pose.pose)*utils::Pose2Transform(dd_pose.pose)*(utils::Pose2Transform(ee_pose_initial.pose)).getRotation(),com_pose.pose.orientation);
                tf2::convert(utils::Pose2Transform(d_pose)*(utils::Pose2Transform(ee_pose_initial.pose)).getRotation(),com_pose.pose.orientation);
                break;

            case 5:
                // FIXED FRAME (BASE_LINK - MIRRORED)
                tmp_pose=d_pose; 
                d_pose.orientation.x=-tmp_pose.orientation.x;
                d_pose.orientation.y=-tmp_pose.orientation.y;
                //d_pose.orientation.z=-tmp_pose.orientation.x;
                dd_pose.pose=utils::Transform2Pose(utils::Pose2Transform(d_pose)); //NEW
                com_pose.pose.position.x =ee_pose_initial.pose.position.x + dd_pose.pose.position.x;
                com_pose.pose.position.y =ee_pose_initial.pose.position.y + dd_pose.pose.position.y;
                com_pose.pose.position.z =ee_pose_initial.pose.position.z + dd_pose.pose.position.z;
                tf2::convert(utils::Pose2Transform(d_pose)*(utils::Pose2Transform(ee_pose_initial.pose)).getRotation(),com_pose.pose.orientation);
            
            case 6:
                // PALETTA FRAME (MIRRORED)
                tf2::convert((utils::Pose2Transform(d_pose)*utils::Pose2Transform(ee_pose_initial.pose)).getRotation(),
                       com_pose.pose.orientation);
                tmp_pose=d_pose; 
                d_pose.orientation.x=-tmp_pose.orientation.x;
                d_pose.orientation.y=-tmp_pose.orientation.z;
                d_pose.orientation.z=-tmp_pose.orientation.y;

                com_pose.pose.position.x=ee_pose_initial.pose.position.x+d_pose.position.x;
                com_pose.pose.position.y=ee_pose_initial.pose.position.y+d_pose.position.y;
                com_pose.pose.position.z=ee_pose_initial.pose.position.z+d_pose.position.z;

                com_pose.pose=utils::Transform2Pose(utils::Pose2Transform(ee_pose_initial.pose)*utils::Pose2Transform(d_pose)); //NEW

                break;
            }
        
        
        //com_pose.pose.position.x=camera_pose.pose.position.x+d_pose.position.x;
        //com_pose.pose.position.y=camera_pose.pose.position.y+d_pose.position.y;
        //com_pose.pose.position.z=camera_pose.pose.position.z+d_pose.position.z;

        com_pose.header.stamp=ros::Time::now();
        com_pose.header.frame_id="base_link";
        pub_robot_com.publish(com_pose);
     
    } else{
      com_pose.pose = ee_pose.pose;
      pub_robot_com.publish(com_pose);
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "haptic_soma");
    //Haptic *hap=new Haptic;
    haptic_guidance gui;//=new haptic_guidance();<--
    Haptic hap(gui); //<--

    ros::Rate rate(100);
    //    drdStop();
    //dhdSetGravityCompensation(DHD_ON);
    gui.init(); //<--
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        //hap->haptic_loop();

        hap.haptic_loop(gui);
    }
    //delete hap;
    return 0;

}