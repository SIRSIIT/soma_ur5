/*
 * controller.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: Joao Bimbo
 */

#include <soma_ur5/controller.h>


UR5_Control::UR5_Control(){
    this->nh=new ros::NodeHandle();
    jo={2,1,0,3,4,5}; //'elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
    sub_joints = nh->subscribe("joint_states", 1000, &UR5_Control::joint_state_callback, this);
    pub_ee_pose = nh->advertise<geometry_msgs::Pose>("ee_pose",5);
    ROS_INFO("Loading parameters...");
    nh->getParam("limits/workspace", map_ws_lim);
    nh->getParam("limits/joints", map_j_lim);
    nh->getParam("control_topic", control_topic);
    nh->getParam("limits/max_angle", max_angle);

    act_client=new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(*nh,control_topic);    
    act_client->waitForServer();
    ROS_INFO("client is up");
    sub_goal_pose= nh->subscribe("goal_pose", 1000, &UR5_Control::goal_pose_callback, this);

}

bool UR5_Control::send_joint_command(double j_com[6]){

    control_msgs::FollowJointTrajectoryGoal jt;
    trajectory_msgs::JointTrajectoryPoint jp;
    jt.trajectory.header.stamp=ros::Time::now()+ros::Duration(0.1);
    jt.trajectory.joint_names=cur_joints.name;

    jp.positions=cur_joints.position;
    jp.velocities=std::vector<double>(6,0);
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
            ROS_WARN_STREAM(print_matrix(4,4,T[i],"Pose (invalid)"));
            return false;
        }
    }

    return true;
}
//Joint Callback
void UR5_Control::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg){
    cur_joints=*msg;

    double q[6];
    double pos[16];

    //save joint_state
    for(int i=0;i<6;i++) {
        q[i]=msg->position.at(jo[i]);
    }

    ur_kinematics::forward(q,pos);

    Tb_ee.setBasis(tf::Matrix3x3(
                       pos[0],pos[1],pos[2],
            pos[4],pos[5],pos[6],
            pos[8],pos[9],pos[10]));
    Tb_ee.setOrigin(tf::Vector3(pos[3],pos[7],pos[11]));
    tf_br.sendTransform(tf::StampedTransform(Tb_ee,ros::Time::now(),"base_link","ee_frame"));

    geometry_msgs::Pose ee_pose;
    geometry_msgs::Quaternion ee_orien;

    ee_pose.position.x=pos[3];
    ee_pose.position.y=pos[7];
    ee_pose.position.z=pos[11];

    tf::quaternionTFToMsg(Tb_ee.getRotation(),ee_orien);
    ee_pose.orientation=ee_orien;

    pub_ee_pose.publish(ee_pose);

    ROS_DEBUG_STREAM_THROTTLE(1,print_matrix(4,4,pos,"EE_pose"));

}

std::string UR5_Control::print_matrix(int m,int n, double* M, std::string prefix) {
    std::ostringstream dbg_msg;
    dbg_msg.setf(std::ios_base::fixed);
    dbg_msg << prefix << std::setprecision(4) << "\n";
    for(int i=0;i<m;i++){
        for(int j=0;j<n;j++){
            dbg_msg <<  M[i*m+j] <<  "\t";
        }
        dbg_msg <<  "\n";
    }
    dbg_msg <<  "\n";
    return dbg_msg.str();
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
                if((cur_j_array[j]<-PI && sols[i*6+j]>-0.1) || (sols[i*6+j]>PI && cur_j_array[j]<0.1)){
                    sols[i*6+j]-=2*PI;
                }
                else if((cur_j_array[j]>PI && sols[i*6+j]<0.1) || ( sols[i*6+j]<-PI && cur_j_array[j]>-0.1)){
                    sols[i*6+j]+=2*PI;
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

void UR5_Control::goal_pose_callback(const geometry_msgs::Pose::ConstPtr &msg){

    std::clock_t    start     = std::clock();
    ROS_DEBUG_STREAM("T1: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000));
    double T_pose[16];
    double sols[36];
    tf::Quaternion tf_quat;

    tf::quaternionMsgToTF(msg->orientation,tf_quat);
    tf::Transform Tg;
    Tg.setRotation(tf_quat);
    Tg.setOrigin(tf::Vector3(msg->position.x,msg->position.y,msg->position.z));

    // Tg=Tb_ee; //Try giving the current pose to inverse kinematics
    tf::Vector3 el,dv;
    dv=Tg.getOrigin();
    el=Tg.getBasis().getRow(0);
    T_pose[0]=el.getX();
    T_pose[1]=el.getY();
    T_pose[2]=el.getZ();
    T_pose[3]=dv.getX();
    el=Tg.getBasis().getRow(1);
    T_pose[4]=el.getX();
    T_pose[5]=el.getY();
    T_pose[6]=el.getZ();
    T_pose[7]=dv.getY();
    el=Tg.getBasis().getRow(2);
    T_pose[8]=el.getX();
    T_pose[9]=el.getY();
    T_pose[10]=el.getZ();
    T_pose[11]=dv.getZ();
    T_pose[12]=0;
    T_pose[13]=0;
    T_pose[14]=0;
    T_pose[15]=1;

    ROS_DEBUG_STREAM("T2: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000));

    ROS_DEBUG_STREAM(print_matrix(4,4,T_pose,"Goal_el:"));


    int num_sols=ur_kinematics::inverse(T_pose,sols);

    if(num_sols==0){
        ROS_ERROR("No solution");
        return;
    }

    double best_sol[6];
    double max_cost;
    if(choose_sol(num_sols,sols,best_sol,max_cost)){
        ROS_DEBUG_STREAM("T3: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000));
        ROS_INFO_STREAM("Moving to: " << best_sol[0] << ","<< best_sol[1] << ","<< best_sol[2] << ","<< best_sol[3] << ","<< best_sol[4] << ","<< best_sol[5] << " max_cost: " << max_cost);
        if(max_cost<max_angle) send_joint_command(best_sol);
        else ROS_ERROR("Target too far away");
        ROS_DEBUG_STREAM("T4: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000));
    }
    else {
        ROS_ERROR("No valid solution");
        return;
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


