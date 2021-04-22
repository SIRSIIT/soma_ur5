#!/usr/bin/env python

import roslib; roslib.load_manifest('soma_ur5')
import rospy
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, TwistStamped, Twist, WrenchStamped, Vector3, Wrench, Quaternion
from object_recognition_msgs.msg import Table
from visualization_msgs.msg import MarkerArray
from copy import deepcopy
import tf2_ros
import tf2_geometry_msgs
from IPython import embed
import math
from tf import transformations as tt
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger
from std_msgs.msg import Header, Float32MultiArray, Int16MultiArray
import moveit_commander
import moveit_msgs.msg
import sys
from ur5MoveScoop import UR5MoveScoop


class GMRScoopGrasp:
    def __init__(self):
        self.move=UR5MoveScoop()
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(120.0))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        #self.got_poses=False        
        #self.box_poses=PoseArray()
        #self.table=Table()
        #self.bbox= MarkerArray()
        self.fingers= Int16MultiArray()
        self.rate = rospy.Rate(100.0)

        self.sub_pre_grasp=rospy.Subscriber("/fingers",Int16MultiArray,self.fingers_callback, queue_size=1)
        #self.sub_table=rospy.Subscriber("/table",Table,self.table_callback, queue_size=1)
        #self.sub_box_dim=rospy.Subscriber("/bounding_boxes",MarkerArray,self.bbox_callback, queue_size=1)
        self.pub_pose_dbg=rospy.Publisher("/ur5/goal_pose",PoseStamped,queue_size=1) #/ur5/goal_pose
        self.pub_fingers=rospy.Publisher("/cmd_fingers",Int16MultiArray,queue_size=1)

        #self.pub_box_dbg=rospy.Publisher("/dbg_bbox",MarkerArray,queue_size=100)
        self.base_link='base_link'
        
        self.pre_grasp= Pose()


    def fingers_callback(self,data):
        self.fingers=data.data;
        
        #self.fingers.data.insert(0,[-self.pose[0][6],self.pose[0][7]])
        #self.pub_fingers.publish(self.fingers)
        
        '''
        pre_grasp.pose.position.x = data[0] 
        pre_grasp.pose.position.y = data[1]
        pre_grasp.pose.position.z = data[2]
        pre_grasp.pose.orientation.x = data[3]
        pre_grasp.pose.orientation.y = data[4]
        pre_grasp.pose.orientation.z = data[5]
        pre_grasp.pose.orientation.w = data[6]
        '''

    def bbox_callback(self,data):
        self.bbox=data
        self.pub_pose_dbg.publish(self.obj_pose)
        self.pub_box_dbg.publish(self.obj_bbox)

    def get_param_from_matlab_cb(self,param):
        self.param=param.data

    def goto_initial(self):
        #joint_goal = [0.0, 0.0, 0.0, -1.69, 0.0, 1.46, 0.76]
        joint_goal = [1.1381, -1.6652, 1.5010, 0.2597, -5.0440, 0.0175]
        self.move.UR5GotoJoint(joint_goal,flag=0)               

    def goto_pregrasp(self,pose):
        self.move.UR5GotoPlan(pose)


    def traj_grasp(self):       
        #self.obj_pose.header=self.box_poses.header
        sub_traj_matlab=rospy.Subscriber("/abs_traj",Float32MultiArray,self.get_param_from_matlab_cb, queue_size=2)
        print('waiting for matlab...')
        rospy.wait_for_message('/abs_traj',Float32MultiArray)
        print('received')
        traj=np.reshape(self.param,(len(self.param)/7,7))


        p=PoseStamped()
        #p_trans=PoseStamped()
        '''
        p.pose.position.x = traj[0,0]
        p.pose.position.y = traj[0,1]
        p.pose.position.z = traj[0,2]
        p.pose.orientation.x = traj[0,3]
        p.pose.orientation.y = traj[0,4]
        p.pose.orientation.z = traj[0,5]
        p.pose.orientation.w = traj[0,6]

        
        trans=None
        while trans is None:
            try:
                trans = self.tfBuffer.lookup_transform('ee_link','ur5_paletta_link', rospy.Time(),rospy.Duration(2.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue
        '''

        R = tt.quaternion_matrix([-0.499, -0.501, -0.499, 0.501])
        t = tt.translation_matrix([0, -0.033, -0.272])
        T = tt.concatenate_matrices(t,R)


        PR = tt.quaternion_matrix([traj[0,3], traj[0,4], traj[0,5],traj[0,6]])
        Pt = tt.translation_matrix([traj[0,0],traj[0,1],traj[0,2]])
        PT = tt.concatenate_matrices(Pt,PR)

        TT = PT.dot(T)

        new_pose_quaternion = tt.quaternion_from_matrix(TT)

        p.pose.orientation.x = new_pose_quaternion[0] 
        p.pose.orientation.y = new_pose_quaternion[1] 
        p.pose.orientation.z = new_pose_quaternion[2] 
        p.pose.orientation.w = new_pose_quaternion[3] 

        p.pose.position.x = TT[0,3]
        p.pose.position.y = TT[1,3]
        p.pose.position.z = TT[2,3]

        p.header.frame_id = "base_link"
        p.header.stamp = rospy.Time.now()
        '''
        print('ciao', TT)        
        p_trans.header = p.header
        p_trans = tf2_geometry_msgs.do_transform_pose(p, trans)
        
        '''
        #self.pub_pose_dbg.publish(p)
        
        if not self.move.UR5GotoPlan(p.pose):
            return
        
        self.rate.sleep()
        goal_pose=PoseStamped()
        goal_pose.header.frame_id = "base_link"
        
        #t=raw_input('Go? (press y)')
        #    if(t == 'y'):

        for i in range(1617/7-1):
            
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.pose.position.x = traj[i,0]
            goal_pose.pose.position.y = traj[i,1]
            goal_pose.pose.position.z = traj[i,2]
            goal_pose.pose.orientation.x = traj[i,3]
            goal_pose.pose.orientation.y = traj[i,4]
            goal_pose.pose.orientation.z = traj[i,5]
            goal_pose.pose.orientation.w = traj[i,6]
            

            self.pub_pose_dbg.publish(goal_pose)
            self.rate.sleep()

        self.move.grasp_object("c")
        rospy.sleep(6.0)

        #print('',len(traj))
        for i in range(1617/7,len(traj)):
            
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.pose.position.x = traj[i,0]
            goal_pose.pose.position.y = traj[i,1]
            goal_pose.pose.position.z = traj[i,2]
            goal_pose.pose.orientation.x = traj[i,3]
            goal_pose.pose.orientation.y = traj[i,4]
            goal_pose.pose.orientation.z = traj[i,5]
            goal_pose.pose.orientation.w = traj[i,6]
            

            self.pub_pose_dbg.publish(goal_pose)
            self.rate.sleep()

        #embed()

                    
    def run(self):
        while not rospy.is_shutdown():
            t=raw_input('''Go?
            '1': go to initial position;                       
            '2': start grasp;  
            '3': go to pre grasp;        
            'b': sensor bias
            'h': configuration setting;
            'o': open the gripper;
            'c': close the gripper;
            'e': embed();
            ''')
            if(t=='1'):
                self.goto_initial() 
            elif(t=='2'):
                self.traj_grasp()  
            elif(t=='3'):
                self.goto_pregrasp(pre_grasp) 
            elif(t=='h'):
                self.finger_config() 
            elif(t=='e'):                
                embed()
            elif(t=='o'):
                self.move.grasp_object("o")
            elif(t=='c'):
                self.move.grasp_object("c")
            else:
                print('wrong command')

def main():
    rospy.init_node('GMRScoop_Grasp')
    tg=GMRScoopGrasp()    
    tg.run();

if __name__ == '__main__':
    main();