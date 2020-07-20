//**********************************//
//   Close = C or c
//   Open  = O or o
//   Print data = P or p
//   No Print = N or n

//  Gripper will stop closing once it detects a certain threshold (contact)
//  Sending closing signal again will increase the torque of the gripper as well initiate closing

//**********************************//

#include <ax12.h>
#include "ros.h"
#include <ros/time.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
bool print_data = false;               // True for Printing Statistics must be used with FTDI Cable
bool print_status = true;             //  Only Print status of the gripper, Can work on XBEE and FTDI both

const int led = 0;
const int bt1 = 1;                    // Push Button to Close
const int bt2 = 2;                    // Push button to Open
const int handle_bt1 = 22;            // Push Button for Handle-Close
const int handle_bt2 = 23;            // Push Button for Handle-Open

const int SERVO_R = 1;                // Finger Side Servo ID
const int SERVO_L = 2;                // Paletta Side Servo ID
unsigned int rstart = 2500;           // Servo inital Position (Finger Side)
unsigned int lstart = 3050;           // Servo Initial Position (Paletta Side)
unsigned int rstop = 800;             // Servo Final Position (Finger Side)
unsigned int lstop = 2000;            // Servo Final Position (Paletta Side)
unsigned int pos[2];                  // Integers to save motors positions
unsigned int buff[2];                 // Buffer
unsigned int send_pos;
unsigned int gposR;
unsigned int gposL;
unsigned int load[2];                  //Integers to save motors load
unsigned int loadR;
unsigned int loadL;
unsigned int close_vel_fing = 50;      // Fingers closing speed
unsigned int open_vel = 50;            // Gripper open speed
unsigned int close_vel_pal = 40;       // Paletta Close Speed
unsigned int max_torque = 1023;        // Motors Max Torque
unsigned int torq_thresh = 550;        // Torque Threshold to read Contact
unsigned int inc_torq = 450;
unsigned int inc_factor = 10;          // Torque Increased Factor
unsigned int loadThresh=0;
float load_percentR=0.0;
float load_percentL=0.0;
int flag=0;
long start_time=0;
long stop_time=0;
bool time_set=false;
bool send_load=false;

bool bclose = false;
bool bopen = false;
bool contact = false;
float rec = 2500;
//String rec = "o";
//String gripper_state = "FULL EXTENDED";
String gripper_state = " ";

geometry_msgs::WrenchStamped state;
ros::NodeHandle nh;
ros::Publisher pub_ax12("/motor_status", &state);

void callback(const std_msgs::Float32& msg)
{
  rec = msg.data;
}

ros::Subscriber<std_msgs::Float32> sub("/cmd_gripper", &callback);

void sub_main_code(float, unsigned int*);       // Function containing Main Code of the Gripper
void DynaSet(unsigned int, unsigned int, unsigned int, unsigned int); // Function to set position DynaSet(ID,Position,Velocity,Torque)
void Read_pos(unsigned int, unsigned int); // Function to read positions of motors Read_pos(ID,ID)
//String Read_gripper_state();              // Function to read Gripper State
void Read_load(unsigned int, unsigned int);// Function to read motors load Read_load(ID,ID)
//void print_all();
void states();



void setup() {
  dxlInit(1000000);  // initialize dynamixel at 1MBPS,
  pinMode(led, OUTPUT);
  pinMode(bt1, INPUT_PULLUP);
  pinMode(bt2, INPUT_PULLUP);
  pinMode(handle_bt1, INPUT_PULLUP);
  pinMode(handle_bt2, INPUT_PULLUP);
  DynaSet(SERVO_R, rstart, open_vel, max_torque);
  DynaSet(SERVO_L, lstart, open_vel, max_torque);

//  Serial.begin(57600); //115200
  

  nh.initNode();
  nh.advertise(pub_ax12);
  nh.subscribe(sub);

  //Serial.print("**** Welcome To Paletta Hand ****** ");

}


void loop() {
  state.header.stamp = nh.now();
  
  /*---------------------------------------------------------------------------------------------------------------------------------*/
  /*  We defined a geometry_msgs of type wrench in order to send position and torque data of, respectively, fingers and scoop motor  */
  /*---------------------------------------------------------------------------------------------------------------------------------*/
  buff[0]=gposR;
  state.wrench.force.x = gposR;         /*   Fingers motor position  */
  state.wrench.force.y = gposL;         /*    Scoop motor position   */
  load_percentR = (loadR*2.5)/1023;     /*   Fingers motor torque    */
  load_percentL = (loadL*2.5)/1023;     /*    Scoop motor torque     */
  state.wrench.torque.x = load_percentR;
  state.wrench.torque.y = load_percentL;

  pub_ax12.publish(&state);
  nh.spinOnce();
  delay(50);
  sub_main_code(rec,*buff);
}
