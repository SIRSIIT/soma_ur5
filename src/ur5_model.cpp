#include <soma_ur5/ur5_model.h>

UR5_Model::UR5_Model(ros::NodeHandle nh_in)
//    : params_{ros::NodeHandle("~")}
{
    dynamic_reconfigure::Server<soma_ur5::dyn_ur5_modelConfig>::CallbackType f;
    f=boost::bind(&UR5_Model::reconfigureRequest, this, _1, _2);
    config_server.setCallback(f);

    //params_= new soma_ur5::dyn_ur5_modelParameters({ros::NodeHandle("~")});
    //ROS_INFO("A");
    //params_->fromParamServer();
    //ROS_INFO("B");

    //    currents_to_torques << 1.0000 ,   0.0435,
    //                           0.0763 ,   0.5031,
    //                           0.1014 ,  -0.0076,
    //                           0.5753 ,  -0.1368,
    //                           0.1742 ,  -0.0147,
    //                           1.0000 ,  -0.0190;


    //    currents_to_torques << 1.0000  ,  0.0111,
    //                           0.0390  ,  1.1750,
    //                           0.0418  ,  0.6124,
    //                           0.0619  ,  0.0523,
    //                           0.1052  ,  0.0655,
    //                           0.0344  ,  0.0463;

    //currents_to_torques <<    10.0000 ,   0.0000,
    //                          13.9373 ,  -0.6318,
    //                          12.5725 ,   0.9007,
    //                           3.5255 ,   0.2833,
    //                           0.6054 ,  -0.0445,
    //                          -3.2312 ,   0.1146;


    currents_to_torques <<    13.2 ,   0.0000,
            13.2, 0.0000,
            13.2,   0.0000,
            9.3,  0.0000,
            9.3, 0.0000,
            9.3 ,  0.0000;


    for(int i=0;i<6;i++){
        cur_filters.push_back(LP_Filter(100));
    }

    //this->nh=new ros::NodeHandle();
    this->nh=&nh_in;

    init=false;    using_gazebo=false; using_hand=true;

    cur_joints.name.resize(6);cur_joints.position.resize(6);
    cur_joints.velocity.resize(6);cur_joints.effort.resize(6);

    sub_joints = nh->subscribe("joint_states", 1000, &UR5_Model::joint_state_callback, this);
    pub_joint_torque = nh->advertise<std_msgs::Float64MultiArray>("kdl_torque",5);

    pub_joint_kdl = nh->advertise<sensor_msgs::JointState>("kdl_joints",5);
    pub_kdl_pose = nh->advertise<geometry_msgs::PoseStamped>("ee_pose",10);



    kdl_parser::treeFromParam("/robot_description",robot_tree_w_hand);
    kdl_parser::treeFromParam("robot_description",robot_tree);

    robot_tree_w_hand.getChain("base_link","soft_hand_palm_link",robot_chain_w_hand);
    KDL::SegmentMap segmap=robot_tree.getSegments();

    for(KDL::SegmentMap::iterator a=segmap.begin();a!=segmap.end();a++){

        ROS_INFO("NAME: %s",a->second.segment.getName().c_str());
    }


    double Hand_mass=0.620;
    //double ati_gamma_mass=0.255;
    double ati_gamma_mass=0.5;
    //    KDL::Vector Hand_cog(0.01,0,0.12);
    KDL::Vector Hand_cog(0.06,0,0.0);

    KDL::RigidBodyInertia Hand_inertia=KDL::RigidBodyInertia(Hand_mass+ati_gamma_mass,Hand_cog,Cube_Rot_Inertia(Hand_mass,0.08,0.12,0.2));
    robot_tree.addSegment(KDL::Segment("hand",KDL::Joint(),KDL::Frame(
                                           KDL::Rotation(0.0,0.0,1.0,
                                                         0,1,0,
                                                         -1,0,0),KDL::Vector(0.05,0.0,0.0)),Hand_inertia),"ee_link");


    robot_tree.getChain("base_link","hand",robot_chain);


    for (int i=0;i<8;i++){
        ROS_WARN("%s",robot_chain.getSegment(i).getName().c_str());
    }


    Jac_solver=new KDL::ChainJntToJacSolver(robot_chain);

    inv_solver= new KDL::ChainIkSolverPos_LMA(robot_chain);
    fksolv=new KDL::ChainFkSolverPos_recursive(robot_chain);

    while(!init){
        ros::spinOnce();
        ros::Rate(10).sleep();
    }
    ROS_INFO("GO!");
    if(using_gazebo)   {
        if(!using_hand){
            jo={2,1,0,3,4,5}; //'elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        }
        else{
            jo={2,1,0,4,5,6}; //name: ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'soft_hand_synergy_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        }
    }

    else jo={0,1,2,3,4,5};



    speed_command = nh->advertise<trajectory_msgs::JointTrajectory>("ur_driver/joint_speed",5);
    nh->getParam("limits/workspace", map_ws_lim);
    nh->getParam("limits/joints", map_j_lim);
    nh->getParam("control_topic", control_topic);
    nh->getParam("limits/max_angle", max_angle);
    nh->getParam("limits/max_speed", max_speed);
    sub_goal_pose= nh->subscribe("goal_pose", 1000, &UR5_Model::goal_pose_callback, this);
}

void UR5_Model::reconfigureRequest(soma_ur5::dyn_ur5_modelConfig& config, uint32_t level) {
    params.speed_gain=config.speed_gain;
    ROS_ERROR("New speed: %f",params.speed_gain);
}





Vector6d UR5_Model::fwd_kin(double q[6]){
    double r,p,y;
    Vector6d fw;

    KDL::JntArray joint_pos=KDL::JntArray(6);
    KDL::Frame cart_pos;
    for (int i=0;i<6;i++){
        joint_pos(i)=q[i];
    }

    fksolv->JntToCart(joint_pos,cart_pos);

    double T_j[16];
    cart_pos.Make4x4(T_j);
    tf2::Matrix3x3(T_j[0],T_j[1],T_j[2],
            T_j[4],T_j[5],T_j[6],
            T_j[8],T_j[9],T_j[10]).getRPY(r,p,y);
    fw << T_j[3], T_j[7], T_j[11], r, p, y;
    return fw;

}


Vector6d UR5_Model::getDeltaX(geometry_msgs::Pose cur,geometry_msgs::Pose goal){
    tf2::Transform T_cur=utils::Pose2Transform(cur);
    tf2::Transform T_goal;
    double rc,pc,yc,rg,pg,yg;
    Vector6d delta_x;

    T_goal=utils::Pose2Transform(goal);
    T_cur.getBasis().getRPY(rc,pc,yc);
    T_goal.getBasis().getRPY(rg,pg,yg);
    delta_x  << T_goal.getOrigin().getX()-T_cur.getOrigin().getX(),
            T_goal.getOrigin().getY()-T_cur.getOrigin().getY(),
            T_goal.getOrigin().getZ()-T_cur.getOrigin().getZ(),
            utils::constrainAngle(rg-rc),utils::constrainAngle(pg-pc),utils::constrainAngle(yg-yc);
    return delta_x;
}

Matrix6d UR5_Model::getJacobian(sensor_msgs::JointState j){
    Matrix6d J;
    Vector6d cur_c,nc;
    double cur_q[j.name.size()],next_q[j.name.size()];
    double h=0.001;

    for(int i=0;i<6;i++) cur_q[i]=j.position.at(i);
    cur_c=fwd_kin(cur_q);

    for(int i=0;i<6;i++) {
        for(int j=0;j<6;j++) {
            next_q[j]=cur_q[j];
        }
        next_q[i]+=h;
        nc=fwd_kin(next_q);
        J.col(i) = 1/h*(nc-cur_c).col(0);
    }
    return J;
}

trajectory_msgs::JointTrajectory UR5_Model::calcSpeeds(geometry_msgs::Pose cur_pose, geometry_msgs::Pose goal_pose,double gain){
    trajectory_msgs::JointTrajectory traj;
    Vector6d deltaX,deltaTh;
    Matrix6d Jacobian;

    deltaX=getDeltaX(cur_pose,goal_pose);
    Jacobian=getJacobian(cur_joints);
    /*
    KDL::JntArray joints=KDL::JntArray(robot_chain.getNrOfJoints());
    for(int i=0;i<robot_chain.getNrOfJoints();i++){
        joints(i)=cur_joints.position.at(i);
    }

    calculateJacobian(joints,Jacobian);
*/


    //if(Jacobian.determinant()<0.001) deltaTh=safeFwdKin(cur_pose,goal_pose);
    // else deltaTh=utils::pseudoinv(Jacobian)*deltaX;

    deltaTh=utils::pseudoinv(Jacobian)*deltaX;


    trajectory_msgs::JointTrajectoryPoint p;
    for(int i=0;i<cur_joints.name.size();i++){
        traj.joint_names.push_back(cur_joints.name.at(i));
    }
    for(int i=0;i<cur_joints.name.size();i++){
        p.velocities.push_back(deltaTh[i]*gain);
    }

    //    ROS_INFO_STREAM("Jacobian:\n" << Jacobian);

    for(int i=0;i<6;i++){
        ROS_INFO("deltaTh[%d]=%f",i,deltaTh[i]);
    }

    ROS_WARN_STREAM("\n==DeltaX=\n==\n" << deltaX << "\n===\n");

    for(int i=0;i<6;i++){
        if(fabs(deltaX[i])>1.0){
            ROS_ERROR("X %d %f",i,deltaX[i]);
            p=prev_vel;
        }
    }

    prev_vel=p;
    traj.header.stamp=ros::Time::now();
    traj.points.push_back(p);
    return traj;
}

trajectory_msgs::JointTrajectory  UR5_Model::safety_enforcer( trajectory_msgs::JointTrajectory in){

    trajectory_msgs::JointTrajectory out=in;
    //check joint limits:
    Vector6d min_limits,max_limits;
    min_limits << map_j_lim["j0_min"],map_j_lim["j1_min"],map_j_lim["j2_min"],map_j_lim["j3_min"],map_j_lim["j4_min"],map_j_lim["j5_min"];
    max_limits << map_j_lim["j0_max"],map_j_lim["j1_max"],map_j_lim["j2_max"],map_j_lim["j3_max"],map_j_lim["j4_max"],map_j_lim["j5_max"];


    for(int i=0;i<6;i++){
        ROS_DEBUG("Lim(%d)=[%f,%f] - (%f)",i,min_limits(i),max_limits(i),cur_joints.position.at(i));

        if(cur_joints.position.at(i)>max_limits(i) && out.points.at(0).velocities.at(i)>0){
            out.points.at(0).velocities.at(i)=0;
            ROS_ERROR("Out of limits [joint %d] > %f",i,max_limits(i));
        }
        if(cur_joints.position.at(i)<min_limits(i) && out.points.at(0).velocities.at(i)<0){
            out.points.at(0).velocities.at(i)=0;
            ROS_ERROR("Out of limits [joint %d] < %f",i,min_limits(i));
        }
    }

    for (int i=0;i<in.points.size();i++){
        for (int j=0;j<in.points.at(i).velocities.size();j++){
            out.points.at(i).velocities.at(j)=std::max(std::min(max_speed,
                                                                out.points.at(i).velocities.at(j)
                                                                ),-max_speed);
        }
    }
    return out;
}

void UR5_Model::goal_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    trajectory_msgs::JointTrajectory vels;
    KDL::JntArray joints=KDL::JntArray(robot_chain.getNrOfJoints());

    for(int i=0;i<robot_chain.getNrOfJoints();i++){
        joints(i)=cur_joints.position.at(i);
    }
 //   vels=calcSpeeds(getEEpose(joints),msg->pose,params.speed_gain);


    KDL::Frame kdl_goal;
    KDL::JntArray target_joints;
    tf::poseMsgToKDL(msg->pose,kdl_goal);
    inv_solver->CartToJnt(joints,kdl_goal,target_joints);
    KDL::Jacobian Jac;
    Jac_solver->JntToJac(joints,Jac);



    for(int i=0;i<cur_joints.name.size();i++){
        vels.joint_names.push_back(cur_joints.name.at(i));
    }
    vels.points.resize(1);
    vels.points.at(0).velocities.resize(6);
    for(int i=0;i<robot_chain.getNrOfJoints();i++){
        vels.points.at(0).velocities.at(i)=(target_joints(i)-joints(i))*params.speed_gain;
    }

    ROS_WARN("Joints: %f %f %f %f %f %f",target_joints(0),target_joints(1),target_joints(2),target_joints(3),target_joints(4),target_joints(5));
    speed_command.publish(safety_enforcer(vels));
}

void UR5_Model::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg){

    if(!init || jo[0]>5){
        if(msg->name.at(0)=="elbow_joint") using_gazebo=true;
        else using_gazebo=false;
        if(msg->name.size()==7) using_hand=true;
        else using_hand=false;
        init=true;
        return;
    }
    //cur_joints=*msg;

    cur_joints.header=msg->header;
    for(long unsigned int i=0;i<jo.size();i++){
        cur_joints.name.at(i)=msg->name.at(jo[i]);
        cur_joints.position.at(i)=msg->position.at(jo[i]);
        cur_joints.velocity.at(i)=msg->velocity.at(jo[i]);
        cur_joints.effort.at(i)=msg->effort.at(jo[i]);
    }
    ROS_INFO("Number of joints: %d",robot_chain.getNrOfJoints());

    KDL::JntArray joint_pos = KDL::JntArray(robot_chain.getNrOfJoints());
    //save joint_state
    for(int i=0;i<robot_chain.getNrOfJoints();i++) {
        joint_pos(i)=msg->position.at(jo[i]);
    }

    geometry_msgs::PoseStamped kdl_pose;
    kdl_pose.pose=getEEpose(joint_pos);
    kdl_pose.header.stamp=ros::Time::now();
    kdl_pose.header.frame_id="base_link";
    pub_kdl_pose.publish(kdl_pose);
    getGravityTorques(joint_pos);
}


KDL::RotationalInertia UR5_Model::Cube_Rot_Inertia(double m,double w, double h, double d){
    return KDL::RotationalInertia((1/12)*m*((h*h)+d*d),
                                  (1/12)*m*((w*w)+d*d),
                                  (1/12)*m*((w*w)+h*h));
}

geometry_msgs::Pose UR5_Model::getEEpose(KDL::JntArray joint_pos){
    KDL::Frame cart_pos;
    geometry_msgs::Pose pose;

    fksolv->JntToCart(joint_pos,cart_pos);

    double EE_Mat[16];
    cart_pos.Make4x4(EE_Mat);
    ROS_INFO("EE: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f",
             EE_Mat[0],EE_Mat[1],EE_Mat[2],EE_Mat[3],
            EE_Mat[4],EE_Mat[5],EE_Mat[6],EE_Mat[7],
            EE_Mat[8],EE_Mat[9],EE_Mat[10],EE_Mat[11],
            EE_Mat[12],EE_Mat[13],EE_Mat[14],EE_Mat[15]);

    pose.position.x=cart_pos.p.x();
    pose.position.y=cart_pos.p.y();
    pose.position.z=cart_pos.p.z();
    cart_pos.M.GetQuaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
    return pose;
}

bool UR5_Model::calculateJacobian(KDL::JntArray in, Matrix6d &J){
    KDL::Jacobian Jac=KDL::Jacobian(6);
    Jac_solver->JntToJac(in,Jac);
    J=Jac.data;
    return true;

}

KDL::JntArray UR5_Model::getGravityTorques(KDL::JntArray joint_pos){

    KDL::Vector g(0,0,-9.8);
    /*KDL::ChainDynParam DynPar= KDL::ChainDynParam(robot_chain_w_hand,g);
    KDL::JntArray grav_torq = KDL::JntArray(robot_chain_w_hand.getNrOfJoints());

    DynPar.JntToGravity(joint_pos,grav_torq);
    ROS_INFO("JT_g: %f %f %f %f %f %f",grav_torq(0),grav_torq(1),grav_torq(2)
             ,grav_torq(3),grav_torq(4),grav_torq(5));

*/

    KDL::ChainDynParam DynPar2= KDL::ChainDynParam(robot_chain,g);
    KDL::JntArray grav_torq2 = KDL::JntArray(robot_chain.getNrOfJoints());
    DynPar2.JntToGravity(joint_pos,grav_torq2);
    ROS_INFO("JT_g2: %f %f %f %f %f %f",grav_torq2(0),grav_torq2(1),grav_torq2(2)
             ,grav_torq2(3),grav_torq2(4),grav_torq2(5));

    sensor_msgs::JointState j_kdl;
    j_kdl.header.stamp=ros::Time::now();
    for(int i=0;i<robot_chain.getNrOfJoints();i++){
        cur_filters.at(i).add_measurement(cur_joints.effort.at(i));
        j_kdl.name.push_back(cur_joints.name.at(i));
        j_kdl.position.push_back(cur_joints.position.at(i));
        j_kdl.effort.push_back(grav_torq2(i)-cur_filters.at(i).get_average()*currents_to_torques(i,0));
    }
    pub_joint_kdl.publish(j_kdl);


    return grav_torq2;
    /*
    ROS_INFO("DOF: %d %d",robot_chain_w_hand.getNrOfJoints(),robot_chain.getNrOfJoints());

    int nn=7;
    ROS_INFO("%f %s %f || %s %f",
             joint_pos(1),
             robot_chain_w_hand.getSegment(nn).getName().c_str(),
             robot_chain_w_hand.getSegment(nn).getInertia().getMass(),
             robot_chain.getSegment(nn).getName().c_str(),
             robot_chain.getSegment(nn).getInertia().getMass());


    sensor_msgs::JointState j_kdl;
    std_msgs::Float64MultiArray j_array;
    j_kdl.header.stamp=ros::Time::now();
    for(int i=0;i<robot_chain.getNrOfJoints();i++){
        cur_filters.at(i).add_measurement(cur_joints.effort.at(i));
        j_kdl.position.push_back(joint_pos(i));
        //j_kdl.position.push_back(cur_filters.at(i).get_average());
        //j_kdl.velocity.push_back(cur_joints.velocity.at(i));
        //j_kdl.velocity.push_back(grav_torq2(i)-cur_filters.at(i).get_average()*currents_to_torques(i,0)+currents_to_torques(i,1));
        j_kdl.velocity.push_back(cur_filters.at(i).get_average()*currents_to_torques(i,0));
        j_kdl.effort.push_back(grav_torq2(i));

    }
    pub_joint_torque.publish(j_array);
    pub_joint_kdl.publish(j_kdl);
    */
}

void UR5_Model::run(){
    // KDL::JntArray joint_pos = KDL::JntArray(robot_chain.getNrOfJoints());

    //for (int i=0;i<robot_chain.getNrOfJoints();i++){
    //    joint_pos(i)=cur_joints.position.at(i);
    // }
    //  calculateJacobian(joint_pos);
    ROS_INFO_STREAM("P: " << params.speed_gain);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "ur5_model");
    ros::NodeHandle nh;
    ros::Rate rate(50);
    UR5_Model *ur5=new UR5_Model(nh);
    Vector6d q;
    while(ros::ok()){
        ros::spinOnce();
        //   ur5->getGravityTorques(q);
        ur5->run();
        rate.sleep();
    }
    return 0;
}
