#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('soma_ur5')
import rospy
import actionlib
import math
import copy
import numpy
from geometry_msgs.msg import PoseStamped, WrenchStamped,Vector3,Quaternion
from qb_interface.msg import handRef

class SOMA_ICRA:
    def __init__(self):
        self.is_running=False;
        rospy.init_node("SOMA_ICRA", anonymous=True, disable_signals=True)
        rospy.Subscriber("ee_pose", PoseStamped,self.p_callback)
        rospy.Subscriber("/netft_data", WrenchStamped,self.f_callback)
        self.goal_pub = rospy.Publisher('goal_pose', PoseStamped, queue_size=1)     
        self.grasp_pub = rospy.Publisher('/qb_class/hand_ref', handRef, queue_size=10)     

        self.initial_force=WrenchStamped()
        self.cur_force=WrenchStamped()
        self.cur_pose=PoseStamped()
        self.rate=rospy.Rate(20)
        while self.cur_pose.pose.orientation.w==0 or self.cur_force.wrench.force.z==0:
            self.rate.sleep();
            print(self.cur_pose.pose.orientation.w)
            print(self.cur_force.wrench.force)
            print("___")
            print(self.initial_force.wrench.force.z)
            
        rospy.Rate(1).sleep();
        self.initial_pose=copy.deepcopy(self.cur_pose);
        self.initial_force=copy.deepcopy(self.cur_force);
        
        self.inc_quat=Quaternion();
        self.inc_quat.w=0.9999905;
        #self.inc_quat.w=0.99905;
        self.inc_quat.x=0.0;
        self.inc_quat.y=0.0;
        self.inc_quat.z=0.0043633;
        #self.inc_quat.z=0.04362;
        self.sq2=math.sqrt(2)/2;
        #self.inc_rot=numpy.matrix('0.9996 0.0283 0;-0.0283 0.9996 0;0 0 1.0000');
   
    def quaternion_multiply(self,q1, q0):
        ret_q=Quaternion();
        ret_q.x=q1.x*q0.w + q1.y*q0.z - q1.z*q0.y + q1.w*q0.x;
        ret_q.y=-q1.x*q0.z + q1.y*q0.w + q1.z*q0.x + q1.w*q0.y;
        ret_q.z=q1.x*q0.y - q1.y*q0.x + q1.z*q0.w + q1.w*q0.z;
        ret_q.w=-q1.x*q0.x - q1.y*q0.y - q1.z*q0.z + q1.w*q0.w;
        return ret_q;
    
    def quaternion_conjugate(self,q):
        ret_q=Quaternion();
        ret_q.x=-q.x;
        ret_q.y=-q.y;
        ret_q.z=-q.z;
        ret_q.w=q.w;
        return ret_q;
            
    # def quat2transform(self,q):
    #     xx2 = 2 * q.x * q.x
    #     yy2 = 2 * q.y * q.y
    #     zz2 = 2 * q.z * q.z
    #     xy2 = 2 * q.x * q.y
    #     wz2 = 2 * q.w * q.z
    #     zx2 = 2 * q.z * q.x
    #     wy2 = 2 * q.w * q.y
    #     yz2 = 2 * q.y * q.z
    #     wx2 = 2 * q.w * q.x
        
    #     rmat = numpy.empty((3, 3), float)
    #     rmat[0,0] = 1. - yy2 - zz2
    #     rmat[0,1] = xy2 - wz2
    #     rmat[0,2] = zx2 + wy2
    #     rmat[1,0] = xy2 + wz2
    #     rmat[1,1] = 1. - xx2 - zz2
    #     rmat[1,2] = yz2 - wx2
    #     rmat[2,0] = zx2 - wy2
    #     rmat[2,1] = yz2 + wx2
    #     rmat[2,2] = 1. - xx2 - yy2
        
    #     return rmat
    
    # def mat2quat(self,M):
    #     # Qyx refers to the contribution of the y input vector component to
    #     # the x output vector component.  Qyx is therefore the same as
    #     # M[0,1].  The notation is from the Wikipedia article.
    #     Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = M.flat
    #     # Fill only lower half of symmetric matrix
    #     K = numpy.array([
    #         [Qxx - Qyy - Qzz, 0,               0,               0              ],
    #         [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0              ],
    #         [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0              ],
    #         [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
    #         ) / 3.0
    #     # Use Hermitian eigenvectors, values for speed
    #     vals, vecs = numpy.linalg.eigh(K)
    #     # Select largest eigenvector, reorder to w,x,y,z quaternion
    #     q = vecs[[3, 0, 1, 2], numpy.argmax(vals)]
    #     # Prefer quaternion with positive w
    #     # (q * -1 corresponds to same rotation as q)
    #     if q[0] < 0:
    #         q *= -1
            
    #     qq=Quaternion();
    #     qq.w=q[0];
    #     qq.x=q[1];
    #     qq.y=q[2];
    #     qq.z=q[3];
    #     return qq
    
    

    def p_callback(self,msg):
        self.cur_pose=msg;

    def f_callback(self,msg):
        self.cur_force=msg;
        self.cur_force.wrench.force.x=self.cur_force.wrench.force.x-self.initial_force.wrench.force.x;
        self.cur_force.wrench.force.y=self.cur_force.wrench.force.y-self.initial_force.wrench.force.y;
        self.cur_force.wrench.force.z=self.cur_force.wrench.force.z-self.initial_force.wrench.force.z;
        if self.initial_force.wrench.force.z is not 0.0:
            self.is_running=True;

    
    def norm(self,obj):
        mag=math.sqrt(obj.x*obj.x+obj.y*obj.y+obj.z*obj.z);
        return mag
        
    def diff(self,v1,v2):
        d=Vector3();
        d.x=v1.x-v2.x
        d.y=v1.y-v2.y
        d.z=v1.z-v2.z
        return self.norm(d)
   
    def z_angle(self,mat):
        return numpy.arccos(mat[1,1]);
    
    
    def wrap_angle(self,ang):
        newAngle=ang;
        while (newAngle <= -math.pi):
            newAngle = newAngle + 2*math.pi;
        while (newAngle > math.pi):
            newAngle -= 2*math.pi;
        return newAngle;

    
    def quat_angle(self,q1,q2):
        qr=self.quaternion_multiply(q1,self.quaternion_conjugate(q2));
        return numpy.fabs(self.wrap_angle(2*numpy.arccos(qr.w)))
        
    
    def go_down(self,pose,m,f,inc):
        starting_pose=copy.deepcopy(pose);
        com_pose=copy.deepcopy(starting_pose);
        while(self.norm(self.cur_force.wrench.force) < f and self.diff(self.cur_pose.pose.position,starting_pose.pose.position) < m):
            rospy.loginfo("dist: %f",self.diff(self.cur_pose.pose.position,starting_pose.pose.position));
            com_pose.pose.position.z=com_pose.pose.position.z+inc;
            self.goal_pub.publish(com_pose)
            rospy.loginfo("%f ||  %f",self.diff(self.cur_pose.pose.position,starting_pose.pose.position),self.norm(self.cur_force.wrench.force))
            self.rate.sleep();
        return True
         
        
    # def rotate(self,pose,g_rot):
    #     starting_pose=copy.deepcopy(pose);
    #     com_pose=copy.deepcopy(starting_pose);
    #     print(numpy.rad2deg(self.quat_angle(self.cur_pose.pose.orientation,starting_pose.pose.orientation)))

    #     tot_rot=numpy.identity(3);
    #     tot_quat=Quaternion();
    #     tot_quat.w=1.0;
    #     if g_rot<0:
    #         self.inc_quat.z=-0.0043633;
    #     while(self.quat_angle(self.cur_pose.pose.orientation,starting_pose.pose.orientation) < numpy.fabs(g_rot)):
    #         rospy.loginfo("%f %f",self.quat_angle(self.cur_pose.pose.orientation,starting_pose.pose.orientation),numpy.fabs(g_rot));
    #         self.diff(self.cur_pose.pose.orientation,starting_pose.pose.orientation)
    #         com_pose.pose.orientation=self.quaternion_multiply(self.inc_quat,com_pose.pose.orientation);
    #         tot_quat=self.quaternion_multiply(self.inc_quat,tot_quat);
    #         #com_or=self.quat2transform(com_pose.pose.orientation);
    #         #com_or=self.inc_rot*com_or;
    #         #com_pose.pose.orientation=self.mat2quat(com_or)
    #         #tot_rot=self.inc_rot*tot_rot;
    #         #print(tot_rot)
    #         self.goal_pub.publish(com_pose)
    #         self.rate.sleep();

    #     ca=numpy.cos(g_rot);
    #     sa=numpy.sin(g_rot);
    #     m_r=numpy.matrix([[ca,sa,0],[-sa,ca,0],[0,0,1]]);
    #     print(m_r)
        
    #     com_pose.pose.orientation=self.mat2quat(m_r*self.quat2transform(com_pose.pose.orientation));
    #     while(self.quat_angle(self.cur_pose.pose.orientation,starting_pose.pose.orientation) < numpy.fabs(g_rot) or self.diff(self.cur_pose.pose.position,starting_pose.pose.position) > 0.01):
    #         self.goal_pub.publish(com_pose)
    #         self.rate.sleep();
        
    #     return True
        
    def grasp(self,perc):
        clos=handRef();
        clos.closure.append(perc)
        self.grasp_pub.publish(clos)
        return True;
        
    def goto_pose(self,pose):
        while(self.quat_angle(self.cur_pose.pose.orientation,pose.pose.orientation) > 0.1 or self.diff(self.cur_pose.pose.position,pose.pose.position) > 0.01):
            rospy.loginfo("%f %f", self.quat_angle(self.cur_pose.pose.orientation,pose.pose.orientation),self.diff(self.cur_pose.pose.position,pose.pose.position))
            self.goal_pub.publish(pose)
            self.rate.sleep();
        return True;
    
    def run(self):
        self.grasp(0.0);
        raw_input("Let's go! Press Enter to continue...")
        
        poses=PoseStamped();
        poses.header.frame_id='base_link';
        poses.pose.position.x=-0.01;
        poses.pose.position.y=-0.675;
        poses.pose.position.z=0.2;
                    

        # Values for the orientation: 
        # 0, pi/16, pi/8, 3pi/16, pi/4, 5pi/16, 6pi/16, 7pi/16, pi/2, 9pi/16, 10pi/16, 11pi/16, 12pi/16
        # 0, 11.25, 22.5, 33.75, 45, 56.25, 67.5, 78.75, 90, 101.25, 112.5, 123.75, 135

        orientation=numpy.matrix([[self.sq2, 0, -self.sq2, 0], \
        [0.693519922661074,   0.137949689641472,  -0.693519922661074,   0.137949689641472],\
        [0.653281482438188,   0.270598050073098,  -0.653281482438188,   0.270598050073098],\
        [0.587937801209679,   0.392847479193551,  -0.587937801209679,   0.392847479193551],\
        [0.500000000000000,   0.500000000000000,  -0.500000000000000,   0.500000000000000],\
        [0.392847479193551,   0.587937801209679,  -0.392847479193551,   0.587937801209679],\
        [0.270598050073099,   0.653281482438188,  -0.270598050073099,   0.653281482438188],\
        [0.137949689641472,   0.693519922661074,  -0.137949689641472,   0.693519922661074],\
        [0.000000000000000,   0.707106781186547,  -0.000000000000000,   0.707106781186547]]);

        orientation=numpy.matrix([[0.703701868763191 ,  0.069308584599546 , -0.703701868763191 ,  0.069308584599546],\
         [0.676659000587176,   0.205262263761179 , -0.676659000587176 ,  0.205262263761179],\
         [0.623612506493336,   0.333327829238873 , -0.623612506493336 ,  0.333327829238873],\
         [0.546600933500879,   0.448583793171318 , -0.546600933500879 ,  0.448583793171318],\
         [0.448583793171318,   0.546600933500879 , -0.448583793171318 ,  0.546600933500879],\
         [0.333327829238873,   0.623612506493336 , -0.333327829238873 ,  0.623612506493336],\
         [0.205262263761179,   0.676659000587177 , -0.205262263761179 ,  0.676659000587177],\
         [0.069308584599546,   0.703701868763192 , -0.069308584599546 ,  0.703701868763192]]);

        orientation=numpy.matrix([[self.sq2, 0, -self.sq2, 0], \
        [0.693519922661074,   0.137949689641472,  -0.693519922661074,   0.137949689641472],\
        [0.653281482438188,   0.270598050073098,  -0.653281482438188,   0.270598050073098],\
        [0.587937801209679,   0.392847479193551,  -0.587937801209679,   0.392847479193551],\
        [0.500000000000000,   0.500000000000000,  -0.500000000000000,   0.500000000000000],\
        [0.392847479193551,   0.587937801209679,  -0.392847479193551,   0.587937801209679],\
        [0.270598050073099,   0.653281482438188,  -0.270598050073099,   0.653281482438188],\
        [0.137949689641472,   0.693519922661074,  -0.137949689641472,   0.693519922661074],\
        [0.000000000000000,   0.707106781186547,  -0.000000000000000,   0.707106781186547]]);

        for j in range(0,6):
            for i in range(0,1):
                print("go_down");
                poses.header.stamp=rospy.Time.now();
                poses.pose.orientation.w=orientation[j,3];
                poses.pose.orientation.x=orientation[j,0];
                poses.pose.orientation.y=orientation[j,1];
                poses.pose.orientation.z=orientation[j,2];
                
                if(self.goto_pose(poses)):
                    rospy.loginfo(poses)
                    rospy.loginfo(self.cur_pose)
                    raw_input("Press Enter to continue...")
            
                if self.go_down(self.cur_pose,0.2,5.0,-0.002):
                    rospy.loginfo("%d %d",i,j)
                    raw_input("Press Enter to continue...")

                if self.grasp(0.9):
                    rospy.loginfo("Grasped")
                    raw_input("Press Enter to continue...")

                if self.go_down(self.cur_pose,0.1,1005.0,0.002):
                    rospy.loginfo("%d %d up",i,j)
                    raw_input("Press Enter to continue...")

                if self.grasp(0.0):
                    rospy.loginfo("Release")
                    raw_input("Press Enter to continue...")
                
            print("rotate");
            #self.rotate(self.cur_pose,numpy.deg2rad(2.5))
            #raw_input("Rotated. Press Enter to continue...")
        print("done");
        

def main():
    foo=SOMA_ICRA();
    while not foo.is_running:
        rospy.Rate(20).sleep();
        print("waiting...")
    foo.run();   


if __name__ == '__main__': main()


