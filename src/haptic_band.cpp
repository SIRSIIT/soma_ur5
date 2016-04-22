#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>
#include <soma_ur5/chb.h>
#include <qb_interface/handCurrent.h>

class Hap_Band{
public:
    Hap_Band(){
        const char * device = "/dev/ttyACM0";  // Linux
        nh=new ros::NodeHandle();
        force_sub=nh->subscribe("/netft_data",100,&Hap_Band::force_callback,this);
        grip_sub=nh->subscribe("/qb_class/hand_current",100,&Hap_Band::grip_callback,this);
        fd=band.chbInit(device);
        rate= new ros::Rate(50);
        Kf=200;
        Kt=1000;
        Kg=1;
        start_pos=6500;

    }

    void force_to_motors(geometry_msgs::Wrench in, unsigned short mot[5]){

        double fx,fy,tz;
        fx=Kf*in.force.x;
        fy=Kf*in.force.y;
        tz=Kt*in.torque.z;

        mot[0]=(unsigned short) -fy + tz + start_pos;
        mot[1]=(unsigned short) -fx + tz + start_pos;
        mot[2]=(unsigned short) fy + tz + start_pos;
        mot[3]=(unsigned short) fx + tz + start_pos;
        mot[4]=(unsigned short) grip_feedback*Kg;
    }

    void run(){
        ros::spinOnce();

        unsigned short r[5];
        force_to_motors(force_cur.wrench,r);

        band.chbSetPos(fd,3,r[0]);
        band.chbSetPos(fd,2,r[1]);
        band.chbSetPos(fd,1,r[2]);
        band.chbSetPos(fd,5,r[3]);
        band.chbSetPos(fd,0,r[4]);



        ROS_INFO("%d %d %d %d ",r[0], r[1],r[2],r[3]);
        rate->sleep();

    }

protected:
    ChbDev band;
    ros::NodeHandle *nh;
    ros::Subscriber force_sub,grip_sub;
    ros::Rate *rate;
    int fd;
    geometry_msgs::WrenchStamped force_cur;
    void force_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg){
        force_cur=*msg;
    }
    void grip_callback(const qb_interface::handCurrent::ConstPtr &msg){
        grip_feedback=(double) msg->current[0];
    }

    double Kf,Kt,Kg,start_pos,grip_feedback;

};



int main(int argc, char **argv){
    ros::init(argc, argv, "haptic_band");
Hap_Band chb;
while (ros::ok()) {

    chb.run();
}

            return 0;

}


