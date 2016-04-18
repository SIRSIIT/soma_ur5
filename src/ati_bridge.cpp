#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>


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
    tf2_ros::TransformBroadcaster tf_br;
    bool ft_sensor;


    void ee_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
        cur_pose=*msg;
        tf2::Transform Tee;
        tf2::Matrix3x3 Ree;
        geometry_msgs::WrenchStamped Fee;
        if(ft_sensor){
            tf2::convert(msg->pose,Tee);
            tf2::Vector3 vf(cur_force.wrench.force.z,
                            cur_force.wrench.force.y,
                            -cur_force.wrench.force.x);

            tf2::Vector3 vt(cur_force.wrench.torque.z,
                            cur_force.wrench.torque.y,
                            -cur_force.wrench.torque.x);

            Ree=Tee.getBasis();
            vf=Ree*vf;
            vt=Ree*vt;

            geometry_msgs::TransformStamped ee_t;
            ee_t.child_frame_id="ee_t_base";
            ee_t.header.frame_id="base_link";
            ee_t.header.stamp=ros::Time::now();
            ee_t.transform.translation.x=msg->pose.position.x;
            ee_t.transform.translation.y=msg->pose.position.y;
            ee_t.transform.translation.z=msg->pose.position.z;

            ee_t.transform.rotation.w=1.0;
            tf_br.sendTransform(ee_t);


            Fee.wrench.force.x=vf.x()/10;
            Fee.wrench.force.y=vf.y()/10;
            Fee.wrench.force.z=vf.z()/10;
            Fee.wrench.torque.x=vt.x()/10;
            Fee.wrench.torque.y=vt.y()/10;
            Fee.wrench.torque.z=vt.z()/10;

            Fee.header.stamp=ee_t.header.stamp;
            Fee.header.frame_id="ee_t_base";
            pub_forces.publish(Fee);
            }
    }


    void ati_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg){
        cur_force=*msg;
        ft_sensor=true;
    }

};


int main(int argc, char **argv){

    ros::init(argc, argv, "ati_bridge");
    ATI_Bridge *ati=new ATI_Bridge();
    ros::spin();
}
