#include <soma_ur5/ur5_model.h>

UR5_Model::UR5_Model(ros::NodeHandle nh_in)
//    : params_{ros::NodeHandle("~")}
{
    dynamic_reconfigure::Server<soma_ur5::dyn_ur5_modelConfig>::CallbackType f;
    f=boost::bind(&UR5_Model::reconfigureRequest, this, _1, _2);
    config_server.setCallback(f);

    currents_to_torques <<    13.2, 0.0, 13.2, 0.0, 13.2, 0.0,
            09.3, 0.0, 09.3, 0.0, 09.3, 0.0;

    for(int i=0;i<6;i++){
        cur_filters.push_back(LP_Filter(20));
        force_filter.push_back(LP_Filter(20));
    }
    joint_pos = KDL::JntArray(robot_chain.getNrOfJoints());
    hand_weight=1.1;
    hand_gvect=KDL::Vector(0,0,-9.8*hand_weight);
    hand_rvect=KDL::Vector(0,0,0.11);

    //this->nh=new ros::NodeHandle();
    this->nh=&nh_in;
    init=false;    using_gazebo=false; using_hand=true; got_force=false;
    cur_joints.name.resize(6);cur_joints.position.resize(6);
    cur_joints.velocity.resize(6);cur_joints.effort.resize(6);
    pub_joint_torque = nh->advertise<std_msgs::Float64MultiArray>("kdl_torque",5);
    pub_joint_kdl = nh->advertise<sensor_msgs::JointState>("kdl_joints",5);
    pub_kdl_pose = nh->advertise<geometry_msgs::PoseStamped>("ee_pose",10);
    ee_force_pub=nh->advertise<geometry_msgs::WrenchStamped>("ee_force",5);
    pub_hand=nh->advertise<qb_interface::handRef>("/qb_class/hand_ref",5);

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
    KDL::Vector Hand_cog(-0.13,0.3,0.0);

    KDL::RigidBodyInertia Hand_inertia=KDL::RigidBodyInertia(Hand_mass+ati_gamma_mass,Hand_cog,Cube_Rot_Inertia(Hand_mass,0.08,0.12,0.2));
    robot_tree.addSegment(KDL::Segment("hand",KDL::Joint(),KDL::Frame(
                                           KDL::Rotation(0.0,0.0,1.0,
                                                         0,1,0,
                                                         -1,0,0),KDL::Vector(0.20,-0.03,0.0)),Hand_inertia),"ee_link");


    robot_tree.getChain("base_link","hand",robot_chain);


    for (int i=0;i<8;i++){
        ROS_WARN("%s",robot_chain.getSegment(i).getName().c_str());
    }

    Jac_solver=new KDL::ChainJntToJacSolver(robot_chain);

    inv_solver= new KDL::ChainIkSolverPos_LMA(robot_chain);
    fksolv=new KDL::ChainFkSolverPos_recursive(robot_chain);

    sub_joints = nh->subscribe("joint_states", 5, &UR5_Model::joint_state_callback, this);

    cur_joints.name.resize(6);
    joint_pos.resize(6);


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

    for(int i=0;i<cur_joints.name.size();i++){
        vels_to_send.joint_names.push_back(cur_joints.name.at(i));
    }
    vels_to_send.points.resize(1);
    vels_to_send.points.at(0).velocities.resize(6);

    t_last_command=ros::Time::now();


    speed_command = nh->advertise<trajectory_msgs::JointTrajectory>("ur_driver/joint_speed",5);

    nh->getParam("limits/workspace", map_ws_lim);
    nh->getParam("limits/joints", map_j_lim);
    nh->getParam("control_topic", control_topic);
    nh->getParam("limits/max_angle", max_angle);
    nh->getParam("limits/max_speed", max_speed);
    sub_goal_pose= nh->subscribe("goal_pose", 1000, &UR5_Model::goal_pose_callback, this);

    sub_forces = nh->subscribe("/netft_data", 5, &UR5_Model::force_callback, this);
    srv_ft_bias = nh->advertiseService("bias_hand_weight",&UR5_Model::ft_bias_srv, this);

    while(!got_force){
        ros::spinOnce();
        ros::Rate(10).sleep();
    }

    ft_sensor_offset();



    act_srv = new actionlib::ActionServer<soma_ur5::SOMAFrameworkAction>(*nh,"soma_action",false);
    boost::function<void(actionlib::ActionServer<soma_ur5::SOMAFrameworkAction>::GoalHandle)> exec_func_handle,cancel_func_handle;
    exec_func_handle=boost::bind(&UR5_Model::execute_action, this,_1,act_srv);
    cancel_func_handle=boost::bind(&UR5_Model::cancel_action, this,_1,act_srv);
    act_srv->registerGoalCallback(exec_func_handle);
    act_srv->registerCancelCallback(cancel_func_handle);
    act_srv->start();

}
geometry_msgs::Wrench UR5_Model::end_effector_weight(){
    geometry_msgs::Wrench ee_weight;
    KDL::Frame bTs;
    fksolv->JntToCart(joint_pos,bTs);

    KDL::Vector f_ee=bTs.M.Inverse()*hand_gvect;
    KDL::Vector t_ee=hand_rvect*f_ee;
    ee_weight.force.x=f_ee.data[0];
    ee_weight.force.y=f_ee.data[1];
    ee_weight.force.z=f_ee.data[2];
    ee_weight.torque.x=t_ee.data[0];
    ee_weight.torque.y=t_ee.data[1];
    ee_weight.torque.z=t_ee.data[2];

    return ee_weight;

}

void UR5_Model::ft_sensor_offset(){
    geometry_msgs::Wrench ee_weight=end_effector_weight();
    ft_offset=cur_force_raw.wrench;
    ft_offset.force.x-=ee_weight.force.x;
    ft_offset.force.y-=ee_weight.force.y;
    ft_offset.force.z-=ee_weight.force.z;

    ft_offset.torque.x-=ee_weight.torque.x;
    ft_offset.torque.y-=ee_weight.torque.y;
    ft_offset.torque.z-=ee_weight.torque.z;
    ROS_INFO("wh2: %f %f %f | %f %f %f",ee_weight.force.x,ee_weight.force.y,ee_weight.force.z,ee_weight.torque.x,ee_weight.torque.y,ee_weight.torque.z);

}

bool UR5_Model::ft_bias_srv(std_srvs::Empty::Request &req,std_srvs::Empty::Response &rsp){
    ft_sensor_offset();
    return true;
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
   *    KDL::JntArray joints=KDL::JntArray(robot_chain.getNrOfJoints());
   *    for(int i=0;i<robot_chain.getNrOfJoints();i++){
   *        joints(i)=cur_joints.position.at(i);
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

    double max=max_speed;
    for (int i=0;i<in.points.size();i++){
        for (int j=0;j<in.points.at(i).velocities.size();j++){
            //out.points.at(i).velocities.at(j)=std::max(std::min(max_speed,out.points.at(i).velocities.at(j)),-max_speed);
            if(fabs(out.points.at(i).velocities.at(j))>max) max=fabs(out.points.at(i).velocities.at(j));
        }
        for (int j=0;j<in.points.at(i).velocities.size();j++){
            out.points.at(i).velocities.at(j)=out.points.at(i).velocities.at(j)/max*max_speed;
        }
    }
    return out;
}
void UR5_Model::move_pose(const geometry_msgs::Pose goal_pose){
    KDL::Frame kdl_goal;
    KDL::JntArray target_joints;
    tf::poseMsgToKDL(goal_pose,kdl_goal);
    KDL::JntArray joints=KDL::JntArray(robot_chain.getNrOfJoints());
    for(int i=0;i<robot_chain.getNrOfJoints();i++){
        joints(i)=cur_joints.position.at(i);
    }

    inv_solver->CartToJnt(joints,kdl_goal,target_joints);
    for(int i=0;i<robot_chain.getNrOfJoints();i++){
        vels_to_send.points.at(0).velocities.at(i)=(target_joints(i)-joints(i))*params.speed_gain;
    }

    ROS_DEBUG("Joints: %f %f %f %f %f %f",target_joints(0),target_joints(1),target_joints(2),target_joints(3),target_joints(4),target_joints(5));
    speed_command.publish(safety_enforcer(vels_to_send));
}


void UR5_Model::move_twist(const geometry_msgs::Twist goal_twist){
    KDL::Jacobian J(robot_chain.getNrOfJoints());
    KDL::JntArray joints=KDL::JntArray(robot_chain.getNrOfJoints());
    for(int i=0;i<robot_chain.getNrOfJoints();i++){
        joints(i)=cur_joints.position.at(i);
    }
    Jac_solver->JntToJac(joints,J);
    Vector6d dv,dq;
    dv << goal_twist.linear.x,goal_twist.linear.y,goal_twist.linear.z,
            goal_twist.angular.x,goal_twist.angular.y,goal_twist.angular.z;

    dq=utils::pseudoinv(J.data)*dv;
    for(int i=0;i<robot_chain.getNrOfJoints();i++){
        vels_to_send.points.at(0).velocities.at(i)=dq[i];
    }
    speed_command.publish(safety_enforcer(vels_to_send));
}

geometry_msgs::Wrench UR5_Model::wrench_in_base(geometry_msgs::Wrench in){
    geometry_msgs::Wrench wr_base;
    KDL::Frame bTe;
    fksolv->JntToCart(joint_pos,bTe);

    KDL::Vector f_ee(in.force.x,
                     in.force.y,
                     in.force.z);
    KDL::Vector t_ee(in.torque.x,
                     in.torque.y,
                     in.torque.z);

    f_ee=bTe.M*f_ee;
    t_ee=bTe.M*t_ee;

    wr_base.force.x=f_ee[0];wr_base.force.y=f_ee[1];wr_base.force.z=f_ee[2];
    wr_base.torque.x=t_ee[0];wr_base.torque.y=t_ee[1];wr_base.torque.z=t_ee[2];
    return wr_base;
}

void UR5_Model::move_wrench(const geometry_msgs::Wrench goal_wrench){

    KDL::Jacobian J(robot_chain.getNrOfJoints());

    Vector6d delta_t,dq;
    //Vector6d des_t,cur_t;

    Jac_solver->JntToJac(joint_pos,J);


    //cur_t << f_ee[0], f_ee[1], f_ee[2],t_ee[0], t_ee[1], t_ee[2];
    //des_t << goal_wrench.force.x,goal_wrench.force.y,goal_wrench.force.z,
    //       goal_wrench.torque.x,goal_wrench.torque.y,goal_wrench.torque.z;
    //delta_t=(des_t+cur_t);
    geometry_msgs::Wrench cur_wr=wrench_in_base(cur_force.wrench);
    delta_t << goal_wrench.force.x+cur_wr.force.x,
            goal_wrench.force.y+cur_wr.force.y,
            goal_wrench.force.z+cur_wr.force.z,
            goal_wrench.torque.x+cur_wr.torque.x,
            goal_wrench.torque.y+cur_wr.torque.y,
            goal_wrench.torque.z+cur_wr.torque.z;

    delta_t[3]/=100;
    delta_t[4]/=100;
    delta_t[5]/=100;
    //    ROS_DEBUG_STREAM("Force_d\n" << des_t << "\nForce_cur:\n" << cur_t << "\nDeltaT" << delta_t);

    dq=J.data.transpose()*delta_t;
    for(int i=0;i<robot_chain.getNrOfJoints();i++){
        vels_to_send.points.at(0).velocities.at(i)=dq[i]/100;
    }

    ROS_DEBUG("Joints: %.3f %.3f %.3f %.3f %.3f %.3f",
              vels_to_send.points.at(0).velocities.at(0)*1000,
              vels_to_send.points.at(0).velocities.at(1)*1000,
              vels_to_send.points.at(0).velocities.at(2)*1000,
              vels_to_send.points.at(0).velocities.at(3)*1000,
              vels_to_send.points.at(0).velocities.at(4)*1000,
              vels_to_send.points.at(0).velocities.at(5)*1000);
    speed_command.publish(safety_enforcer(vels_to_send));
}

void UR5_Model::goal_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    //   vels=calcSpeeds(getEEpose(joints),msg->pose,params.speed_gain);
    move_pose(msg->pose);
    t_last_command=ros::Time::now();
    stopped=false;

}
void UR5_Model::force_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg){
    cur_force_raw.header=msg->header;
    cur_force.header=msg->header;
    geometry_msgs::Wrench ee_weight;
    ee_weight=end_effector_weight();
    force_filter.at(0).add_measurement(msg->wrench.force.x);
    force_filter.at(1).add_measurement(msg->wrench.force.y);
    force_filter.at(2).add_measurement(msg->wrench.force.z);
    force_filter.at(3).add_measurement(msg->wrench.torque.x);
    force_filter.at(4).add_measurement(msg->wrench.torque.y);
    force_filter.at(5).add_measurement(msg->wrench.torque.z);

    cur_force_raw.wrench.force.x=force_filter.at(0).get_average();
    cur_force_raw.wrench.force.y=force_filter.at(1).get_average();
    cur_force_raw.wrench.force.z=force_filter.at(2).get_average();
    cur_force_raw.wrench.torque.x=force_filter.at(3).get_average();
    cur_force_raw.wrench.torque.y=force_filter.at(4).get_average();
    cur_force_raw.wrench.torque.z=force_filter.at(5).get_average();

    cur_force.wrench.force.x=(cur_force_raw.wrench.force.x-ft_offset.force.x-ee_weight.force.x);
    cur_force.wrench.force.y=(cur_force_raw.wrench.force.y-ft_offset.force.y-ee_weight.force.y);
    cur_force.wrench.force.z=(cur_force_raw.wrench.force.z-ft_offset.force.z-ee_weight.force.z);
    cur_force.wrench.torque.x=(cur_force_raw.wrench.torque.x-ft_offset.torque.x-ee_weight.torque.x);
    cur_force.wrench.torque.y=(cur_force_raw.wrench.torque.y-ft_offset.torque.y-ee_weight.torque.y);
    cur_force.wrench.torque.z=(cur_force_raw.wrench.torque.z-ft_offset.torque.z-ee_weight.torque.z);
    ee_force_pub.publish(cur_force);

    got_force=true;

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
    ROS_DEBUG("Number of joints: %d",robot_chain.getNrOfJoints());

    //save joint_state
    for(int i=0;i<robot_chain.getNrOfJoints();i++) {
        joint_pos(i)=msg->position.at(jo[i]);
    }

    cur_pose.pose=getEEpose(joint_pos);
    cur_pose.header.stamp=ros::Time::now();
    cur_pose.header.frame_id="base_link";
    pub_kdl_pose.publish(cur_pose);
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
    ROS_DEBUG("EE: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f",
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
   *    KDL::JntArray grav_torq = KDL::JntArray(robot_chain_w_hand.getNrOfJoints());
   *
   *    DynPar.JntToGravity(joint_pos,grav_torq);
   *    ROS_INFO("JT_g: %f %f %f %f %f %f",grav_torq(0),grav_torq(1),grav_torq(2)
   *             ,grav_torq(3),grav_torq(4),grav_torq(5));
   *
   */

    KDL::ChainDynParam DynPar2= KDL::ChainDynParam(robot_chain,g);
    KDL::JntArray grav_torq2 = KDL::JntArray(robot_chain.getNrOfJoints());
    DynPar2.JntToGravity(joint_pos,grav_torq2);
    ROS_DEBUG("JT_g2: %f %f %f %f %f %f",grav_torq2(0),grav_torq2(1),grav_torq2(2)
              ,grav_torq2(3),grav_torq2(4),grav_torq2(5));

    sensor_msgs::JointState j_kdl;
    j_kdl.header.stamp=ros::Time::now();
    for(int i=0;i<robot_chain.getNrOfJoints();i++){
        cur_filters.at(i).add_measurement(cur_joints.effort.at(i));
        j_kdl.name.push_back(cur_joints.name.at(i));
        j_kdl.position.push_back(cur_joints.position.at(i));
        //j_kdl.velocity.push_back(grav_torq2(i));
        j_kdl.velocity.push_back(cur_joints.velocity.at(i));
        j_kdl.effort.push_back(grav_torq2(i)-cur_filters.at(i).get_average()*currents_to_torques(i,0));
    }
    pub_joint_kdl.publish(j_kdl);


    return grav_torq2;
    /*
   *    ROS_INFO("DOF: %d %d",robot_chain_w_hand.getNrOfJoints(),robot_chain.getNrOfJoints());
   *
   *    int nn=7;
   *    ROS_INFO("%f %s %f || %s %f",
   *             joint_pos(1),
   *             robot_chain_w_hand.getSegment(nn).getName().c_str(),
   *             robot_chain_w_hand.getSegment(nn).getInertia().getMass(),
   *             robot_chain.getSegment(nn).getName().c_str(),
   *             robot_chain.getSegment(nn).getInertia().getMass());
   *
   *
   *    sensor_msgs::JointState j_kdl;
   *    std_msgs::Float64MultiArray j_array;
   *    j_kdl.header.stamp=ros::Time::now();
   *    for(int i=0;i<robot_chain.getNrOfJoints();i++){
   *        cur_filters.at(i).add_measurement(cur_joints.effort.at(i));
   *        j_kdl.position.push_back(joint_pos(i));
   *        //j_kdl.position.push_back(cur_filters.at(i).get_average());
   *        //j_kdl.velocity.push_back(cur_joints.velocity.at(i));
   *        //j_kdl.velocity.push_back(grav_torq2(i)-cur_filters.at(i).get_average()*currents_to_torques(i,0)+currents_to_torques(i,1));
   *        j_kdl.velocity.push_back(cur_filters.at(i).get_average()*currents_to_torques(i,0));
   *        j_kdl.effort.push_back(grav_torq2(i));
   *
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
    // ROS_INFO_STREAM("P: " << params.speed_gain);
    if((ros::Time::now()-t_last_command).toSec()>0.1 && !stopped){
        if((ros::Time::now()-t_last_command).toSec()>1) {
            stopped=true;
            ROS_WARN("Robot stopped");
        }
        else{
            for(int i=0;i<robot_chain.getNrOfJoints();i++){
                vels_to_send.points.at(0).velocities.at(i)=0;
            }
            speed_command.publish(vels_to_send);
        }
    }
}

bool UR5_Model::monitor_dummy(soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb){
    return false;
}  


bool UR5_Model::monitor_wrench(soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb){        
    if(goal->wrench.force.x!=0){
        if((fabs(goal->wrench.force.x)-fabs(fb->cur_wrench.force.x))<0) return true;
    }
    if(goal->wrench.force.y!=0){
        if((fabs(goal->wrench.force.y)-fabs(fb->cur_wrench.force.y))<0) return true;
    }
    if(goal->wrench.force.z!=0){
        if((fabs(goal->wrench.force.z)-fabs(fb->cur_wrench.force.z))<0) return true;
    }
    if(goal->wrench.torque.x!=0){
        if((fabs(goal->wrench.torque.x)-fabs(fb->cur_wrench.torque.x))<0) return true;
    }
    if(goal->wrench.torque.y!=0){
        if((fabs(goal->wrench.torque.y)-fabs(fb->cur_wrench.torque.y))<0) return true;
    }
    if(goal->wrench.torque.z!=0){
        if((fabs(goal->wrench.torque.z)-fabs(fb->cur_wrench.torque.z))<0) return true;
    }
    return false;
}

bool UR5_Model::monitor_pose(soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb){
    if(empty_msg(goal->pose.position) && empty_msg(goal->pose.orientation)) return false;

    if(goal->pose.position.x!=0){
        if(fabs(goal->pose.position.x-fb->cur_pose.position.x)>0.005) return false;
    }
    if(goal->pose.position.y!=0){
        if(fabs(goal->pose.position.y-fb->cur_pose.position.y)>0.005) return false;
    }
    if(goal->pose.position.z!=0){
        if(fabs(goal->pose.position.z-fb->cur_pose.position.z)>0.005) return false;
    }
    if(goal->pose.orientation.w!=0 || goal->pose.orientation.x!=0 ||
       goal->pose.orientation.y!=0 || goal->pose.orientation.z!=0){
        tf2::Quaternion goal_q(goal->pose.orientation.x,goal->pose.orientation.y,goal->pose.orientation.z,goal->pose.orientation.w);
        tf2::Quaternion cur_q(fb->cur_pose.orientation.x,fb->cur_pose.orientation.y,fb->cur_pose.orientation.z,fb->cur_pose.orientation.w);
        tf2::Quaternion distance_q=cur_q.inverse()*goal_q;
        double da=distance_q.getAngleShortestPath();
        if(da>0.2){
            return false;
        }
    }
    return true;
}

bool UR5_Model::monitor_pose_old(soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb){
    tf2::Vector3 distance_t(goal->pose.position.x-fb->cur_pose.position.x,
                            goal->pose.position.y-fb->cur_pose.position.y,
                            goal->pose.position.z-fb->cur_pose.position.z);

    tf2::Quaternion goal_q(goal->pose.orientation.x,goal->pose.orientation.y,goal->pose.orientation.z,goal->pose.orientation.w);
    tf2::Quaternion cur_q(fb->cur_pose.orientation.x,fb->cur_pose.orientation.y,fb->cur_pose.orientation.z,fb->cur_pose.orientation.w);
    tf2::Quaternion distance_q=cur_q.inverse()*goal_q;

    double dt=distance_t.length();
    double da=distance_q.getAngleShortestPath();
    ROS_WARN("dt: %f da: %f",dt,da);
    if(dt<0.01 && da<0.1)    return true;
    else return false;
}

void UR5_Model::control_force(soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb){
    ROS_INFO("we're controlling force");
    move_wrench(goal->wrench);
}
void UR5_Model::control_position(soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb){
    ROS_INFO("we're controlling position");
    move_pose(goal->pose);
}
void UR5_Model::control_velocity( soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb ){
    ROS_INFO("we're controlling velocity");
    move_twist(goal->twist);
}

void UR5_Model::control_follow( soma_ur5::SOMAFrameworkGoal::ConstPtr goal, soma_ur5::SOMAFrameworkFeedback::Ptr fb ){
    ROS_INFO("we're following");
    move_twist(goal->twist);
}


int UR5_Model::parse_goal(soma_ur5::SOMAFrameworkGoal::ConstPtr g){
    int task=0;

    if(empty_msg(g->wrench.force) && empty_msg(g->wrench.torque)){
        ROS_WARN("No suitable controller");
        task=0;
    }
    else{
        if(empty_msg(g->twist.linear) && empty_msg(g->twist.angular)){
            if(empty_msg(g->pose.position) && empty_msg(g->pose.orientation)){
                ROS_INFO("Admittance (maintain a cart.Force)");
                task=soma_ur5::SOMAFrameworkGoal::FORCE;
            }
            else{
                ROS_INFO("Position Control (go to pose)");
                task=soma_ur5::SOMAFrameworkGoal::POSITION;
            }
        }
        else{
            if(empty_msg(g->pose.position) && empty_msg(g->pose.orientation)){
                ROS_INFO("Velocity Control (maintain a cart. velocity)");
                task=soma_ur5::SOMAFrameworkGoal::VELOCITY;
            }
            else{
                ROS_INFO("Force+Velocity (Follow surface)");
                task=soma_ur5::SOMAFrameworkGoal::FOLLOW;
            }
        }
    }
    return task;
}

void UR5_Model::execute_action(actionlib::ActionServer<soma_ur5::SOMAFrameworkAction>::GoalHandle goal, actionlib::ActionServer<soma_ur5::SOMAFrameworkAction>* as){
    boost::shared_ptr<const soma_ur5::SOMAFrameworkGoal> g=goal.getGoal();
    soma_ur5::SOMAFrameworkFeedback::Ptr fb(new soma_ur5::SOMAFrameworkFeedback());
    soma_ur5::SOMAFrameworkResult res;
    bool stop_exec=false;
    boost::function<bool(soma_ur5::SOMAFrameworkFeedback::Ptr)> monitor_f,achieved_f;
    boost::function<void(soma_ur5::SOMAFrameworkFeedback::Ptr)> exec_controller_f;

    int task=parse_goal(g);

    switch(task){
    case 0:
        task=0;
        res.sequence.push_back(1);
        res.success=false;
        goal.setRejected(res,"No suitable controller");
        return;
    case soma_ur5::SOMAFrameworkGoal::FORCE:
        if(cur_force.header.stamp.sec==0){
            res.sequence.push_back(2);
            res.success=false;
            goal.setRejected(res,"No Force Sensor");
            return;
        }
        monitor_f=boost::bind(&UR5_Model::monitor_dummy, this,g,_1);
        achieved_f=boost::bind(&UR5_Model::monitor_pose, this,g,_1);
        exec_controller_f=boost::bind(&UR5_Model::control_force, this,g,_1);
        break;
    case soma_ur5::SOMAFrameworkGoal::POSITION:
        monitor_f=boost::bind(&UR5_Model::monitor_wrench, this,g,_1);
        achieved_f=boost::bind(&UR5_Model::monitor_pose, this,g,_1);
        exec_controller_f=boost::bind(&UR5_Model::control_position, this,g,_1);
        break;
    case soma_ur5::SOMAFrameworkGoal::VELOCITY:
        monitor_f=boost::bind(&UR5_Model::monitor_wrench, this,g,_1);
        achieved_f=boost::bind(&UR5_Model::monitor_pose, this,g,_1);
        exec_controller_f=boost::bind(&UR5_Model::control_velocity, this,g,_1);
        break;
    case soma_ur5::SOMAFrameworkGoal::FOLLOW:
        if(cur_force.header.stamp.sec==0){
            res.sequence.push_back(2);
            res.success=false;
            goal.setRejected(res,"No Force Sensor");
            return;
        }
        monitor_f=boost::bind(&UR5_Model::monitor_dummy, this,g,_1);
        achieved_f=boost::bind(&UR5_Model::monitor_pose, this,g,_1);
        exec_controller_f=boost::bind(&UR5_Model::control_follow, this,g,_1);
        break;
    }

    if(!stopped) {
        ROS_WARN("Robot moving");
        res.sequence.push_back(10);
        res.success=false;
        goal.setRejected(res,"Robot moving");
        return;
    }


    goal.setAccepted("Executing action");
    ros::Time t_action_start=ros::Time::now();
    qb_interface::handRef grip_ref;
    grip_ref.closure.push_back(g->ellipsoid.x);

    ros::Rate r(50);
    while (goal.getGoalStatus().status==actionlib_msgs::GoalStatus::ACTIVE && (ros::Time::now()-t_action_start).toSec() < g->max_duration){
        fb->cur_wrench=wrench_in_base(cur_force.wrench);
        fb->cur_pose=cur_pose.pose;
        fb->header.stamp=ros::Time::now();
        goal.publishFeedback(*fb);
        ros::spinOnce();
        if(!achieved_f(fb)){
            if(!monitor_f(fb)){
                ROS_INFO("GOOD");
                exec_controller_f(fb);
                pub_hand.publish(grip_ref);
                t_last_command=ros::Time::now();
            }
            else{
                res.sequence.push_back(-1);
                res.success=false;
                goal.setAborted(res,"Failed");
                stop_robot();
            }
        }
        else{
            res.sequence.push_back(1);
            res.success=true;
            goal.setSucceeded(res,"Goal Achieved");
            stop_robot();
        }
        r.sleep();
    }
    stop_robot();
    if(goal.getGoalStatus().status==actionlib_msgs::GoalStatus::ACTIVE && (ros::Time::now()-t_action_start).toSec() > g->max_duration){
        res.sequence.push_back(1);
        res.success=true;
        goal.setSucceeded(res,"timed out");
    }
}

void UR5_Model::stop_robot(){
    for(int i=0;i<robot_chain.getNrOfJoints();i++){
        vels_to_send.points.at(0).velocities.at(i)=0;
    }
    speed_command.publish(vels_to_send);
}

void UR5_Model::cancel_action(actionlib::ActionServer<soma_ur5::SOMAFrameworkAction>::GoalHandle goal, actionlib::ActionServer<soma_ur5::SOMAFrameworkAction>* as){
    boost::shared_ptr<const soma_ur5::SOMAFrameworkGoal> a=goal.getGoal();
    ROS_INFO("BUBUBUBU,%f",a->twist.linear.z);
    soma_ur5::SOMAFrameworkResult res;
    res.success=false;
    res.sequence.push_back(11);
    //goal.setCanceled(res,"cancelled");
    goal.setAborted(res,"aborted");
    stop_robot();
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
