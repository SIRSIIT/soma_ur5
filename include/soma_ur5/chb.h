/*

srvthm.h is a static library made to control the UNISI-Wearable Thimble devices.
This library is opensource and subjected to updates.

contacts: chinello@dii.unisi.it

*/

//#include "serlib.h"
#include <cmath>
#include <iostream>
#include <cstring>
#include <math.h>
// #include "Eigen/Core"
#include <fcntl.h>
#include <unistd.h>

# define PI 3.14159265359
# define sqrt3 1.73205080757

#ifdef _WIN32
#include <Windows.h>
#define O_NOCTTY 0
#else
#include <termios.h>
#endif


//uint8_t po_1; uint8_t po_2; uint8_t po_3;

//using namespace Eigen;
using namespace std;



class ChbDev{
	public:
        
void delay(int time_del){ // time_del millis
      #ifdef _WIN32
        Sleep(time_del);
      #else
        usleep(time_del*1000);
      #endif
    }
int segno(double value){
	int sgn;
		//if(value == 0) sgn = 0;
		if(value > 0) sgn = 1;
		if(value < 0) sgn = -1;
		return sgn;
	}
    		
// Initializes the communication with the Pololu board.
//
//		
		int chbInit(const char * device){ //initializes communication with pololu devices

  		int fd = open(device, O_RDWR | O_NOCTTY);
  		if (fd == -1){
    		
    		cout << "error: " << device << endl;
    		
    		return 1;
  		}

		#ifndef _WIN32
  		struct termios options;
  		tcgetattr(fd, &options);
  		options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  		options.c_oflag &= ~(ONLCR | OCRNL);
  		tcsetattr(fd, TCSANOW, &options);
		#endif
		return fd;
		}

// Gets the position from a channel.
// See the "Serial Servo Commands" section of the user's guide.
//
		int chbRead(int fd, unsigned char channel)
		{
  			unsigned char command[] = {0x90, channel}; // 0x90 commands to read.
		if(write(fd, command, sizeof(command)) == -1){
    		cout << "error: " << "error writing" << endl;
    		return -1;
  		}
   
  			unsigned char response[2];
  			if(read(fd,response,2) != 2){
    		cout << "error: " << "error reading" << endl;
    		return -1;
  		}

  			return response[0] + 256*response[1];
		}
    
// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds.
//

		int chbSetPos(int fd, unsigned char channel, unsigned short target)
		{
  		unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F}; //0x84 write
  		if (write(fd, command, sizeof(command)) == -1){
    		cout << "error: " << "error writing" << endl;
    		return -1;
  		}
  		return 0;
		}	


// set target displacement to reach with the device 
//

		void chbSetDir(int fd, double * polar, double* target){
		
		int pos_a, pos_b, pos_c, pos_d; 
		int motor[4];// = {3, 2, 1, 0};
		if(abs(polar[0]) < abs(target[0])){
		pos_a = polar[0]*(1)*1000.0/target[0] + 6000.0; //

		pos_b = polar[0]*(-1)*1000.0/target[0]  + 6000.0; 
		}
		if(abs(polar[1]) < abs(target[1])){
		pos_c = polar[1]*(1)*1000.0/target[1] + 5500.0;
		pos_d = polar[1]*(-1)*1000.0/target[1] + 6000.0;	
		}
		//cout << pos_c << endl;	
		//	pos_c = 4000; pos_d = 4000;
		// ch = sthChannel(dev);
		// motor={3, 2, 1, 0}

		motor[0] = 3; motor[1] = 2;	motor[2] = 1; motor[3] = 0; 

		// cout << pos_a << endl;
			
			if(abs(polar[0]) < abs(target[0])){	
			 	chbSetPos(fd, motor[0], pos_a);
				chbSetPos(fd, motor[2], pos_b);
			}
			if(abs(polar[1]) < abs(target[1])){
				chbSetPos(fd, motor[1], pos_c);
				chbSetPos(fd, motor[3], pos_d);
			}
		}

// Given a rotation it rotates the pulleys according to the device.
		
		void chbSetRot(int fd, double rotation, double target){
		
		int pos_a, pos_b, pos_c, pos_d; 
		int motor[4];// = {3, 2, 1, 0};

		pos_a = rotation*(-1)*1000.0/target + 6000.0; //

		pos_b = rotation*(1)*1000.0/target  + 6000.0; 

		pos_c = rotation*(1)*1000.0/target + 6000.0;

		pos_d = rotation*(-1)*1000.0/target + 6000.0;	
		
		//cout << pos_c << endl;	
		//	pos_c = 4000; pos_d = 4000;
		// ch = sthChannel(dev);
		// motor={3, 2, 1, 0}

		motor[0] = 3; motor[1] = 2;	motor[2] = 1; motor[3] = 0; 

		// cout << pos_a << endl;
			
			if(abs(rotation) < abs(target)){
			 	chbSetPos(fd, motor[0], pos_a);
				chbSetPos(fd, motor[2], pos_b);
			
				chbSetPos(fd, motor[1], pos_c);
				chbSetPos(fd, motor[3], pos_d);
			}
		}
		
// set target displacement to reach with the device 
//
		void chbSetCue(int fd, double * direction, double* target){
			
			double polar[2]; polar[0] = direction[0]; polar[1] = direction[1];
			double rotation; rotation = direction[2];
			double target_dir[2]; target_dir[0] = target[0]; target_dir[1] = target[1]; 
			double target_rot; target_rot = target[2];
			
			chbSetDir(fd, polar, target_dir);
			chbSetRot(fd, rotation, target_rot);
		
		}		
// set a force whose module and Eulero's angles are known at the center of the platform
//

//void * chbSetRot(int fd, unsigned char dev, double dz, double *force, int c_d){
//  		
//}


//// Reads angles of each motor 4s
////
////
//		double * pdReadPos(int fd, unsigned char dev){
//		
//		static double dev_angles[3]; int pos_a, pos_b, pos_c; int *ch;  
//				
//		int mtr_value_a; int mtr_value_b; int mtr_value_c;

//				ch = sthChannel(dev);
//				
//				dev_angles[0] = sthRead(fd, ch[0])* 90.0/4000.0 - 6000.0*90.0/4000.0;	// from pololu to deg of device
//				dev_angles[1] = sthRead(fd, ch[1])* -90.0/4000.0 + 6000.0*90.0/4000.0;
//				dev_angles[2] = sthRead(fd, ch[2])* 90.0/4000.0 - 6000.0*90.0/4000.0;
//				
//			return dev_angles;	
//		}
		
};


