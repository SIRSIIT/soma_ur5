#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('soma_ur5')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
import pymouse

class ur5_control:
    def __init__(self):
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        self.client = actionlib.SimpleActionClient('/vel_based_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        self.client.wait_for_server()
        print "Connected to server"
        self.joints=JointState();
        rospy.Subscriber("/joint_states", JointState,self.j_callback)
        self.mouse = pymouse.PyMouse()    
        self.rate=rospy.Rate(100)
        self.go=False
        while not self.go:
            self.rate.sleep();
        self.g = FollowJointTrajectoryGoal()
        self.g.trajectory = JointTrajectory()
        self.g.trajectory.joint_names = self.JOINT_NAMES
        

        
    
    def loop(self):
        i_pos=self.mouse.position()
        while not rospy.is_shutdown():
            a=self.mouse.position()            
            ref=i_pos[0]
            inp=a[0]            
            self.rate.sleep();            
            command=self.controller(ref,inp);
            print(command)
            self.send_com(command)


    def controller(self,ref,inp):
        return (ref-inp)/1000.0;
    def send_com(self,comm):
        joints_pos = self.i_joints.position
        joints_pos2 = [joints_pos[0]+comm, joints_pos[1], joints_pos[2], joints_pos[3], joints_pos[4], joints_pos[5]];
        self.g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=joints_pos2, velocities=[0]*6, time_from_start=rospy.Duration(0.1))]	
        self.client.send_goal(self.g)
        self.client.wait_for_result()

    def run(self):
        self.parameters = rospy.get_param(None)        
        try:        
            print "Please make sure that your robot can move freely between these poses before proceeding!"        
            inp = raw_input("Continue? y/n: ")[0]
            if (inp == 'y'):
                self.loop();                                        
            else:
                print "Halting program"

            
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt!")
            #send_com(0.0)
            raise
        


    
    def j_callback(self,msg):
        self.joints=msg;
        self.JOINT_NAMES=msg.name;
        if not self.go:
        	self.go=True
        	self.i_joints=msg;
        #print(msg)

def main():
    foo=ur5_control();
    foo.run();   


if __name__ == '__main__': main()
