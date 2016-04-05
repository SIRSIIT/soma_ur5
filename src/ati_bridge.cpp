#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ATI_Bridge{
public:
    ATI_Bridge(){
        nh=new ros::NodeHandle();
        ft_sensor=false;
        pub_forces=nh->advertise<geometry_msgs::WrenchStamped>("ee_force",5);
        sub_ati= nh->subscribe("/netft_data", 1000, &ATI_Bridge::ati_cb, this);
        sub_ee= nh->subscribe("ee_pose", 1000, &ATI_Bridge::ee_pose_cb, this);

    }
protected:
    ros::NodeHandle *nh;
    ros::Subscriber sub_ati,sub_ee;
    ros::Publisher pub_forces;
    geometry_msgs::PoseStamped cur_pose;
    geometry_msgs::WrenchStamped cur_force;
    bool ft_sensor;


    void ee_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
        cur_pose=*msg;
        tf2::Transform Tee;
        tf2::Matrix3x3 Ree;
        geometry_msgs::WrenchStamped Fee;
        if(ft_sensor){
            tf2::convert(msg->pose,Tee);
            tf2::Vector3 vf(-cur_force.wrench.force.x,
                            -cur_force.wrench.force.y,
                            -cur_force.wrench.force.z);

            tf2::Vector3 vt(-cur_force.wrench.torque.x/1000,
                            -cur_force.wrench.torque.y/1000,
                            -cur_force.wrench.torque.z/1000);

            Ree=Tee.getBasis();
            vf=Ree*vf;
            vt=Ree*vt;


            Fee.wrench.force.x=vf.z();
            Fee.wrench.force.y=vf.y();
            Fee.wrench.force.z=-vf.x();
            Fee.wrench.torque.x=vt.z();
            Fee.wrench.torque.y=vt.y();
            Fee.wrench.torque.z=-vt.x();

            Fee.header.stamp=ros::Time::now();
            Fee.header.frame_id="base_link";
            pub_forces.publish(Fee);
            }
    }


    void ati_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg){
        cur_force=*msg;
        ft_sensor=true;
    }

};


int main(int argc, char **argv){

    ros::init(argc, argv, "haptic_soma");
    ATI_Bridge *ati=new ATI_Bridge();
    ros::spin();
}
