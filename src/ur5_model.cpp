#include <soma_ur5/ur5_model.h>

UR5_Model::UR5_Model(){


    this->nh=new ros::NodeHandle();

    init=false;    using_gazebo=false;

    cur_joints.name.resize(6);cur_joints.position.resize(6);
    cur_joints.velocity.resize(6);cur_joints.effort.resize(6);

    sub_joints = nh->subscribe("joint_states", 1000, &UR5_Model::joint_state_callback, this);
    pub_joint_torque = nh->advertise<std_msgs::Float64MultiArray>("kdl_torque",5);



    kdl_parser::treeFromParam("robot_description",robot_tree);
    robot_tree.getChain("base_link","ee_link",robot_chain);

/*
    double Hand_mass=0.620;
    KDL::Vector Hand_cog(0.01,0,0.08);
    KDL::RigidBodyInertia Hand_inertia=KDL::RigidBodyInertia(Hand_mass,Hand_cog,Cube_Rot_Inertia(Hand_mass,0.08,0.12,0.2));
    robot_chain.addSegment(KDL::Segment("hand",KDL::Joint(),KDL::Frame::Identity(),Hand_inertia));
*/
    while(!init){
       ros::spinOnce();
       ros::Rate(10).sleep();
   }
    ROS_INFO("GO!");
   if(using_gazebo)    jo={2,1,0,3,4,5}; //'elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
   else jo={0,1,2,3,4,5};

}

void UR5_Model::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg){

    if(!init || jo[0]>5){
        if(msg->name.at(0)=="elbow_joint") using_gazebo=true;
        else using_gazebo=false;
        init=true;
        return;
    }
    //cur_joints=*msg;

    cur_joints.header=msg->header;
    for(long unsigned int i=0;i<msg->name.size();i++){
        cur_joints.name.at(i)=msg->name.at(jo[i]);
        cur_joints.position.at(i)=msg->position.at(jo[i]);
        cur_joints.velocity.at(i)=msg->velocity.at(jo[i]);
        cur_joints.effort.at(i)=msg->effort.at(jo[i]);
    }
    ROS_INFO("Initialised 2");

    double q[6];
    double pos[16];
    Vector6d pos_eig;

    //save joint_state
    for(int i=0;i<6;i++) {
        q[i]=msg->position.at(jo[i]);
        pos_eig[i]=msg->position.at(jo[i]);
    }
getGravityTorques(pos_eig);
}


KDL::RotationalInertia UR5_Model::Cube_Rot_Inertia(double m,double w, double h, double d){
    return KDL::RotationalInertia((1/12)*m*((h*h)+d*d),
                           (1/12)*m*((w*w)+d*d),
                           (1/12)*m*((w*w)+h*h));
}

Vector6d UR5_Model::getGravityTorques(Vector6d q){
    Vector6d tj;

    KDL::Frame cart_pos;
    KDL::JntArray joint_pos = KDL::JntArray(robot_chain.getNrOfJoints());
    joint_pos(0)=0.0;
    joint_pos(1)=1.0*KDL::PI;
    joint_pos(2)=0.0;
    joint_pos(3)=0.2;
    joint_pos(4)=0.0;
    joint_pos(5)=0.0;

    for (int i=0;i<robot_chain.getNrOfJoints();i++){
        joint_pos(i)=cur_joints.position.at(i);
    }

    KDL::ChainFkSolverPos_recursive fksolv=KDL::ChainFkSolverPos_recursive(robot_chain);


    fksolv.JntToCart(joint_pos,cart_pos);


    double EE_Mat[16];
    cart_pos.Make4x4(EE_Mat);
    ROS_INFO("EE: %f %f %f",EE_Mat[3],EE_Mat[7],EE_Mat[11]);

    KDL::RigidBodyInertia inertia=robot_chain.getSegment(2).getInertia();
    KDL::Vector cog=inertia.getCOG();
    ROS_INFO("cog: %f %f %f %f",inertia.getMass(),inertia.getCOG().x(),inertia.getCOG().y(),inertia.getCOG().z());


    KDL::Vector g(0,0,-9.8);

    KDL::ChainDynParam DynPar= KDL::ChainDynParam(robot_chain,g);
    KDL::JntArray grav_torq = KDL::JntArray(robot_chain.getNrOfJoints());
    DynPar.JntToGravity(joint_pos,grav_torq);
    ROS_INFO("JT_g: %f %f %f %f %f %f",grav_torq(0),grav_torq(1),grav_torq(2)
             ,grav_torq(3),grav_torq(4),grav_torq(5));

    sensor_msgs::JointState j_kdl;
    std_msgs::Float64MultiArray j_array;
    for(int i=0;i<robot_chain.getNrOfJoints();i++){
        j_kdl.position.push_back(joint_pos(i));
        j_kdl.effort.push_back(grav_torq(i));
        j_array.data.push_back(grav_torq(i)*3.6/50);
    }
pub_joint_torque.publish(j_array);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "ur5_model");
    UR5_Model *ur5=new UR5_Model();
    ros::Rate rate(10);
    Vector6d q;
    while(ros::ok()){
        ros::spinOnce();
     //   ur5->getGravityTorques(q);
        rate.sleep();
    }

    return 0;
}
