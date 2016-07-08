#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('soma_ur5')
import rospy
from sensor_msgs.msg import JointState
import numpy as np

class ExternalDist:
	
	def joint_cb(self,msg):	
		j_torque=JointState();		
		effort=np.zeros(6)
		j_torque.effort=[0,0,0,0,0,0];
		for i in range(0,6):
			effort[i]=msg.position[i] #FIXME			
			j_torque.effort[i]=(self.fitting[i,0]*effort[i]+self.fitting[i,1]);
			print(str(i) + ": " + str(effort[i]) + " -> " + str(j_torque.effort[i]));
		j_torque.header.stamp=rospy.Time.now();
		self.pub_ext.publish(j_torque);		


	def __init__(self):
		print("Bimbo")		
		#self.fitting = [[0 for x in range(6)] for y in range(2)];
		self.fitting=np.array([
			[1, 1.2],
			[40,-40],
			[1,0],
			[1,0],
			[1,0],
			[1,0]]);
		self.pub_ext = rospy.Publisher('external_torque', JointState, queue_size=10)
		self.sub_joints=rospy.Subscriber("kdl_joints",JointState,self.joint_cb)
		rospy.spin();		


def main():
	rospy.init_node("external_dist", anonymous=True, disable_signals=True)
	a=ExternalDist();


if __name__ == '__main__': main()
