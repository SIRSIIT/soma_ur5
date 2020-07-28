#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>


class ATI_Bridge{
public:
    ATI_Bridge(){
        nh=new ros::NodeHandle();
        ft_sensor=false;
        pub_forces=nh->advertise<geometry_msgs::WrenchStamped>("ee_force",5);
        pub_forces_bl=nh->advertise<geometry_msgs::WrenchStamped>("ee_force_blink",5);
        pub_transform=nh->advertise<geometry_msgs::PoseStamped>("ee_t",5);
        sub_ati= nh->subscribe("/ur5/netft_data", 1000, &ATI_Bridge::ati_cb, this);
        sub_ee= nh->subscribe("ee_pose", 1000, &ATI_Bridge::ee_pose_cb, this);

    }
protected:
    ros::NodeHandle *nh;
    ros::Subscriber sub_ati,sub_ee;
    ros::Publisher pub_forces, pub_forces_bl, pub_transform;
    geometry_msgs::PoseStamped cur_pose;
    geometry_msgs::WrenchStamped cur_force, cur_force_in;
    tf2_ros::TransformBroadcaster tf_br;
    geometry_msgs::TransformStamped tf;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    bool ft_sensor;
    bool tf_ok = true;


    void ee_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
        cur_pose=*msg;
        tf2::Transform Tee;
        tf2::Matrix3x3 Ree;
        geometry_msgs::WrenchStamped Fee, FeeBL;
        geometry_msgs::PoseStamped T_pose;
        if(ft_sensor){
            // PALETTA HAND
            try {
                ros::Time now = ros::Time::now();
                listener.waitForTransform("ur5_paletta_link", "ur5_opto_ft_link", now, ros::Duration(0.01));

                listener.lookupTransform("ur5_paletta_link", "ur5_opto_ft_link", ros::Time(0), transform);
                tf.transform.translation.x = transform.getOrigin().x();
                tf.transform.translation.y = transform.getOrigin().y();
                tf.transform.translation.z = transform.getOrigin().z();
                tf.transform.rotation.x = transform.getRotation().x();
                tf.transform.rotation.y = transform.getRotation().y();
                tf.transform.rotation.z = transform.getRotation().z();
                tf.transform.rotation.w = transform.getRotation().w();

            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
                tf_ok = false;
            }
            if (tf_ok) {
                tf2::doTransform(cur_force_in,cur_force,tf);
                cur_force.header.frame_id="ur5_paletta_link";
                cur_force.header.stamp = ros::Time::now();
                //cur_force_filt_x->add_measurement(cur_force.wrench.force.x);
                //cur_force_filt_y->add_measurement(cur_force.wrench.force.y);
                //cur_force_filt_z->add_measurement(cur_force.wrench.force.z);
                //ROS_INFO_STREAM("wrench paletta"<<cur_force_filt_x);
            }
            Fee.header.frame_id=cur_force.header.frame_id;
            Fee.header.stamp =cur_force.header.stamp;
            Fee.wrench.force.x  = cur_force.wrench.force.x/5;
            Fee.wrench.force.y  = cur_force.wrench.force.y/5;
            Fee.wrench.force.z  = cur_force.wrench.force.z/5;
            Fee.wrench.torque.x = cur_force.wrench.torque.x/5;
            Fee.wrench.torque.y = cur_force.wrench.torque.y/5;
            Fee.wrench.torque.z = cur_force.wrench.torque.z/5;
            //pub_forces.publish(Fee);
            

        tf2::convert(msg->pose,Tee);
            // PALETTA TO BASE LINK FT
            tf2::Vector3 vf(Fee.wrench.force.x*5,
                            Fee.wrench.force.y*5,
                            Fee.wrench.force.z*5);

            tf2::Vector3 vt(Fee.wrench.torque.x*5,
                            Fee.wrench.torque.y*5,
                            Fee.wrench.torque.z*5);
            
            /*
            tf2::Vector3 vf(cur_force.wrench.force.z,
                            cur_force.wrench.force.y,
                            -cur_force.wrench.force.x);

            tf2::Vector3 vt(cur_force.wrench.torque.z,
                            cur_force.wrench.torque.y,
                            -cur_force.wrench.torque.x);
            */
            Ree=Tee.getBasis();
            vf=Ree*vf;
            vt=Ree*vt;
            /*
            T_pose.pose.position.x = Tee.getOrigin().x();
            T_pose.pose.position.y = Tee.getOrigin().y();
            T_pose.pose.position.z = Tee.getOrigin().z();
            T_pose.pose.orientation.x = Tee.getRotation().x();
            T_pose.pose.orientation.y = Tee.getRotation().y();
            T_pose.pose.orientation.z = Tee.getRotation().z();
            T_pose.pose.orientation.w = Tee.getRotation().w();
            pub_transform.publish(T_pose);
            */
            /*
            ROS_INFO("Ree: \n%f %f %f\n%f %f %f\n%f %f %f",
            Ree[0],Ree[1],Ree[2],
            Ree[3],Ree[4],Ree[5],
            Ree[6],Ree[7],Ree[8]);

            geometry_msgs::TransformStamped ee_t;
            ee_t.child_frame_id="ee_t_base";
            ee_t.header.frame_id="base_link";
            ee_t.header.stamp=ros::Time::now();
            ee_t.transform.translation.x=msg->pose.position.x;
            ee_t.transform.translation.y=msg->pose.position.y;
            ee_t.transform.translation.z=msg->pose.position.z;

            ee_t.transform.rotation.w=1.0;
            tf_br.sendTransform(ee_t);
            pub_transform.publish(ee_t);
            */ 
            FeeBL.wrench.force.x=vf.x();
            FeeBL.wrench.force.y=vf.y();
            FeeBL.wrench.force.z=vf.z();
            FeeBL.wrench.torque.x=vt.x();
            FeeBL.wrench.torque.y=vt.y();
            FeeBL.wrench.torque.z=vt.z();

            FeeBL.header.stamp=Fee.header.stamp;
            FeeBL.header.frame_id="base_link";
            /*
            T_pose.pose.position.x = Tee.getOrigin().x(); 
            T_pose.pose.position.y = Tee.getOrigin().y(); 
            T_pose.pose.position.z = Tee.getOrigin().z();
            T_pose.pose.orientation.x = Tee.getRotation().x(); 
            T_pose.pose.orientation.y = Tee.getRotation().y(); 
            T_pose.pose.orientation.z = Tee.getRotation().z();
            T_pose.pose.orientation.w = Tee.getRotation().w();
            pub_transform.publish(T_pose);*/
            pub_forces_bl.publish(FeeBL);
            pub_forces.publish(Fee);
        }
    }


    void ati_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg){
        cur_force_in=*msg;
        ft_sensor=true;
    }

};


int main(int argc, char **argv){

    ros::init(argc, argv, "ati_bridge");
    ATI_Bridge *ati=new ATI_Bridge();
    ros::spin();
}
