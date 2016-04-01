/*
 * controller.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: Joao Bimbo
 */

#include <soma_ur5/controller.h>


UR5_Control::UR5_Control(){
    this->nh=new ros::NodeHandle();

    tf_list=new tf2_ros::TransformListener(buffer);
    sub_joints = nh->subscribe("joint_states", 1000, &UR5_Control::joint_state_callback, this);

    init=false;    using_gazebo=false;
    cur_joints.name.resize(6);cur_joints.position.resize(6);
    cur_joints.velocity.resize(6);cur_joints.effort.resize(6);

    pub_ee_pose = nh->advertise<geometry_msgs::PoseStamped>("ee_pose",5);

    dynamic_reconfigure::Server<soma_ur5::dyn_ur5_controllerConfig>::CallbackType f;
    f=boost::bind(&UR5_Control::config_cb, this, _1, _2);
    config_server.setCallback(f);

     while(!init){
        ros::spinOnce();
        ros::Rate(10).sleep();
    }
    if(using_gazebo)    jo={2,1,0,3,4,5}; //'elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
    else jo={0,1,2,3,4,5};
    speed_gain=0;

    speed_command = nh->advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed",5);
    ROS_INFO("Loading parameters...");
    nh->getParam("limits/workspace", map_ws_lim);
    nh->getParam("limits/joints", map_j_lim);
    nh->getParam("control_topic", control_topic);
    nh->getParam("limits/max_angle", max_angle);

    std::string solv_str;
    nh->getParam("solver",solv_str);

    if(boost::iequals(solv_str,"jacobian")) solver=UR5_Control::JACOBIAN;
    else if(boost::iequals(solv_str,"closed_form")) solver=UR5_Control::CLOSED_FORM;


    act_client=new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(*nh,control_topic);
    //act_client->waitForServer();
    //ROS_INFO("client is up");
    sub_goal_pose= nh->subscribe("goal_pose", 1000, &UR5_Control::goal_pose_callback, this);

}

bool UR5_Control::send_speed_command(double j_com[6]){
    trajectory_msgs::JointTrajectory traj;
    traj.points.resize(1);
    traj.header.stamp=ros::Time::now();
    if(speed_gain==0) ROS_WARN_THROTTLE(1.0,"speed_gain is set to zero. Fix: rosrun dynamic_reconfigure dynparam set %s speed_gain 0.01",ros::this_node::getName().c_str());

    for(int i=0;i<6;i++) traj.points.at(0).velocities.push_back(j_com[i]*speed_gain);
    speed_command.publish(traj);
}


bool UR5_Control::send_joint_command(double j_com[6]){

    control_msgs::FollowJointTrajectoryGoal jt;
    trajectory_msgs::JointTrajectoryPoint jp;
    jt.trajectory.header.stamp=ros::Time::now()+ros::Duration(0.1);
    jt.trajectory.joint_names=cur_joints.name;

    jp.positions=cur_joints.position;
    jp.velocities=std::vector<double>(6,0.1);
    jp.accelerations=std::vector<double>(6,0);
    jp.effort=std::vector<double>(6,0);

    jt.trajectory.points.push_back(jp);

    for(int i=0;i<6;i++) jp.positions.at(jo[i])=j_com[i];
    jp.time_from_start=ros::Duration(0.1);

    jt.trajectory.points.push_back(jp);
    //act_client->sendGoal(jt);
    act_client->sendGoalAndWait(jt,ros::Duration(0.4));


}

bool UR5_Control::valid_jconf(double joints[6]){
    double T[6][16];

    if(joints[0]<map_j_lim["j0_min"] || joints[0]>map_j_lim["j0_max"] ||
            joints[1]<map_j_lim["j1_min"] || joints[1]>map_j_lim["j1_max"] ||
            joints[2]<map_j_lim["j2_min"] || joints[2]>map_j_lim["j2_max"] ||
            joints[3]<map_j_lim["j3_min"] || joints[3]>map_j_lim["j3_max"] ||
            joints[4]<map_j_lim["j4_min"] || joints[4]>map_j_lim["j4_max"] ||
            joints[5]<map_j_lim["j5_min"] || joints[5]>map_j_lim["j5_max"]) {

        ROS_WARN("Joint limit exceeded");
        return false;
    }

    ur_kinematics::forward_all(joints,T[0],T[1],T[2],T[3],T[4],T[5]);

    for (int i=0;i<6;i++) {
        if(T[i][3]<map_ws_lim["x_min"] || T[i][3]>map_ws_lim["x_max"] ||
                T[i][7]<map_ws_lim["y_min"] || T[i][7]>map_ws_lim["y_max"] ||
                T[i][11]<map_ws_lim["z_min"] || T[i][11]>map_ws_lim["z_max"]) {
            ROS_WARN("Link %d outside workspace",i);
            ROS_WARN_STREAM(utils::print_matrix(4,4,T[i],"Pose (invalid)"));
            return false;
        }
    }

    return true;
}
//Joint Callback
void UR5_Control::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg){    

    if(!init){
        if(msg->name.at(0)=="elbow_joint") using_gazebo=true;
        else using_gazebo=false;
        init=true;
    }
    //cur_joints=*msg;
    cur_joints.header=msg->header;
    for(int i=0;i<6;i++){
        cur_joints.name.at(i)=msg->name.at(jo[i]);
        cur_joints.position.at(i)=msg->position.at(jo[i]);
        cur_joints.velocity.at(i)=msg->velocity.at(jo[i]);
        cur_joints.effort.at(i)=msg->effort.at(jo[i]);
    }

    double q[6];
    double pos[16];

    //save joint_state
    for(int i=0;i<6;i++) {
        q[i]=msg->position.at(jo[i]);
    }

    ur_kinematics::forward(q,pos);

    Tb_ee.setBasis(tf2::Matrix3x3(
                       pos[0],pos[1],pos[2],
            pos[4],pos[5],pos[6],
            pos[8],pos[9],pos[10]));
    Tb_ee.setOrigin(tf2::Vector3(pos[3],pos[7],pos[11]));


    //tf2::Stamped<tf2::Transform> st(Tb_ee,ros::Time::now(),"base_link","ee_frame");



      //    tf_br.sendTransform(st);

    geometry_msgs::PoseStamped ee_pose;
    geometry_msgs::Quaternion ee_orien;

    ee_pose.pose.position.x=pos[3];
    ee_pose.pose.position.y=pos[7];
    ee_pose.pose.position.z=pos[11];

    //tf::quaternionTFToMsg(Tb_ee.getRotation(),ee_orien);
    tf2::convert(Tb_ee.getRotation(),ee_orien);
    ee_pose.pose.orientation=ee_orien;
    ee_pose.header.stamp=ros::Time::now();
    ee_pose.header.frame_id="base_link";


    pub_ee_pose.publish(ee_pose);

    ROS_INFO_STREAM_THROTTLE(1,utils::print_matrix(4,4,pos,"EE_pose"));

    double T[6][16];
    ur_kinematics::forward_all(q,T[0],T[1],T[2],T[3],T[4],T[5]);
    for (int i=0;i<6;i++) {
      //  ROS_INFO_STREAM(print_matrix(4,4,T[i],"t"+std::to_string(i)+":"));
    }

}



bool UR5_Control::choose_sol(int nsols,double* sols, double* best,double &max_cost){
    double bcost=999;
    double cost=0;
    double cur_j_array[6];
    for(int i=0;i<6;i++) {
        cur_j_array[i]=cur_joints.position[jo[i]];
    }
    for(int i=0;i<nsols;i++) {
        cost=0;
        if(valid_jconf(&sols[i*6])){
            for(int j=0;j<6;j++){
                if((cur_j_array[j]<-M_PI && sols[i*6+j]>-0.1) || (sols[i*6+j]>M_PI && cur_j_array[j]<0.1)){
                    sols[i*6+j]-=2*M_PI;
                }
                else if((cur_j_array[j]>M_PI && sols[i*6+j]<0.1) || ( sols[i*6+j]<-M_PI && cur_j_array[j]>-0.1)){
                    sols[i*6+j]+=2*M_PI;
                }
                cost+=fabs(cur_j_array[j]-sols[i*6+j]);
            }
            if(cost<bcost) {
                memcpy(best,&sols[i*6],6*sizeof(double));
                bcost=cost;
            }
        }
    }
    if(bcost!=999){
        max_cost=0;
        for(int i=0;i<6;i++){
            double c=fabs(cur_j_array[i]-best[i]);
            if(c>max_cost) max_cost=c;
        }
        return true;
    }
    else{
        max_cost=999;
        return false;
    }
}

void UR5_Control::config_cb(soma_ur5::dyn_ur5_controllerConfig &config, uint32_t level) {
    ROS_DEBUG("Reconfigure Request.");
        speed_gain=config.speed_gain;
}


void UR5_Control::goal_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){

    double T_pose[16];
    utils::pose2array(msg->pose,T_pose);

    ROS_DEBUG_STREAM(utils::print_matrix(4,4,T_pose,"Goal_el:"));


    double comm[6];
    switch (solver){
    case(UR5_Control::CLOSED_FORM):
        closed_form(T_pose,comm);
        break;
    case(UR5_Control::JACOBIAN):
        jac_based(T_pose,comm);
        break;
    }
}

Vector6d UR5_Control::fwd_kin(double q[6]){
    double r,p,y;
    Vector6d fw;
    double T_j[16];
    ur_kinematics::forward(q,T_j);
    tf2::Matrix3x3(T_j[0],T_j[1],T_j[2],
            T_j[4],T_j[5],T_j[6],
            T_j[8],T_j[9],T_j[10]).getRPY(r,p,y);
    fw << T_j[3], T_j[7], T_j[11], r, p, y;
    return fw;

}

void UR5_Control::calculate_jac(double cur_q[6], Matrix6d &J){

    double h=0.01;
    double next_q[6];

    Vector6d cur_c,nc;

    cur_c=fwd_kin(cur_q);

    for(int i=0;i<6;i++) {
        for(int j=0;j<6;j++) {
            next_q[j]=cur_q[j];
        }
        next_q[i]+=h;
        nc=fwd_kin(next_q);
        J.col(i) = 1/h*(nc-cur_c).col(0);
    }
    ROS_INFO("Determinant(J): %f",J.determinant());
}

bool UR5_Control::jac_based(double *goal, double *comm){


    tf2::Transform T_cur=Tb_ee;
    tf2::Transform T_goal;
    geometry_msgs::Pose p_tmp;
    double cur_q[6];
    double rc,pc,yc,rg,pg,yg;
    Vector6d delta_x,delta_th;
    double delta_th_array[6];

    for(int i=0;i<6;i++) {
        cur_q[i]=cur_joints.position.at(i);
    }
   // ROS_INFO("cur q: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f",
   //          cur_q[0],cur_q[1],cur_q[2],cur_q[3],cur_q[4],cur_q[5]);

    T_cur.getBasis().getRPY(rc,pc,yc);

    Matrix6d Jac;
    calculate_jac(cur_q,Jac);
 //   ROS_INFO_STREAM("J:" << std::endl << Jac);

    utils::array2pose(goal,p_tmp,T_goal);
    T_goal.getBasis().getRPY(rg,pg,yg);

    delta_x  << T_goal.getOrigin().getX()-T_cur.getOrigin().getX(),
            T_goal.getOrigin().getY()-T_cur.getOrigin().getY(),
            T_goal.getOrigin().getZ()-T_cur.getOrigin().getZ(),
            rg-rc,pg-pc,yg-yc;

 //   ROS_INFO_STREAM("delta_x:" << delta_x);

    delta_th=utils::pseudoinv(Jac)*delta_x;
    //delta_th=Jac.transpose()*delta_x;
//    ROS_INFO_STREAM("delta_th:" << delta_th);

    for (int i=0;i<6;i++) {
        delta_th_array[i]=delta_th(i);
    }
    //if(Jac.determinant()>0)
        send_speed_command(delta_th_array);
}




bool UR5_Control::closed_form(double *goal, double *comm){
    double sols[36];
    int num_sols=ur_kinematics::inverse(goal,sols);
    if(num_sols==0){
        ROS_ERROR("No solution");
        return false;
    }

    double best_sol[6];
    double max_cost;
    if(choose_sol(num_sols,sols,best_sol,max_cost)){
        ROS_INFO_STREAM("Moving to: " << best_sol[0] << ","<< best_sol[1] << ","<< best_sol[2] << ","<< best_sol[3] << ","<< best_sol[4] << ","<< best_sol[5] << " max_cost: " << max_cost);
        if(max_cost<max_angle) comm=best_sol;
        else ROS_ERROR("Target too far away");
    }
    else {
        ROS_ERROR("No valid solution");
        return false;
    }

}

int main(int argc, char **argv){

    ros::init(argc, argv, "ur5_soma");
    UR5_Control *ur5=new UR5_Control();
    ros::Rate rate(100);

    while(ros::ok()){
        ros::spin();
    }

    return 0;
}


