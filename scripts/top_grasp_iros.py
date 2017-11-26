#!/usr/bin/env python

import roslib; roslib.load_manifest('soma_ur5')
import rospy
import smach
import smach_ros
import actionlib
from soma_ur5.msg import *
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
import copy
import tf2_ros
import tf2_geometry_msgs
from IPython import embed
import math
from tf import transformations as tt
import numpy as np

class Initial(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['top_grasp'],
			 input_keys=['object_pose','constraint_pose'])
  def execute(self, userdata):
      rospy.sleep(1);
      return 'top_grasp'


class TopGraspFull:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(120.0))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.got_poses=False;
        self.bTc=None
        self.poses=PoseArray();
        self.rate = rospy.Rate(10.0)
        self.sub_poses=rospy.Subscriber("/poses_boxes",PoseArray,self.poses_callback, queue_size=2)
        self.sub_ee_pose=rospy.Subscriber("ee_pose",PoseStamped,self.ee_pose_cb, queue_size=2)
        self.pub_debug=rospy.Publisher("debug_poses", PoseStamped,queue_size=2)
        while not self.got_poses:
            rospy.loginfo("F: %s",self.poses.header.frame_id);
            self.rate.sleep()
        while self.bTc is None:
            try:
                self.bTc = self.tfBuffer.lookup_transform('base_link',self.poses.header.frame_id, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue
        rospy.loginfo(self.bTc);

    def ee_pose_cb(self,data):
        self.ee_pose=data;

    def a_goto_initial(self):
        goal=SOMAFrameworkGoal();
        goal.controller=soma_ur5.msg.SOMAFrameworkGoal.POSITION;
        goal.max_duration=6.0
        goal.header.frame_id='base_link';
        goal.header.stamp=rospy.Time.now();
        goal.pose=geometry_msgs.msg.Pose();
        goal.pose.position.x=-0.50;
        goal.pose.position.y=-0.25;
        goal.pose.position.z=0.30;
        goal.pose.orientation.w= -0.0;
        goal.pose.orientation.x= -0.65;
        goal.pose.orientation.y= -0.3;
        goal.pose.orientation.z= 0.7;
        goal.wrench.force.z=-10.0;
        return goal;


    def a_goto_approach(self,pose,ell):
        goal=SOMAFrameworkGoal();
        goal.controller=soma_ur5.msg.SOMAFrameworkGoal.POSITION;
        goal.max_duration=10
        goal.header.frame_id='base_link';
        goal.header.stamp=rospy.Time.now();
        goal.pose=self.obj_to_goal_pose(pose,ellipsoid=ell);
        goal.pose.position.z=goal.pose.position.z+0.15;
        goal.wrench.force.z=-4.0;
        return goal;

    def a_approach_down(self,v,t,f,grip):
        goal=SOMAFrameworkGoal();
        goal.controller=soma_ur5.msg.SOMAFrameworkGoal.VELOCITY;
        goal.max_duration=t
        goal.header.frame_id='base_link';
        goal.header.stamp=rospy.Time.now();
        goal.twist.linear.z=v;
        goal.wrench.force.z=f;
        goal.ellipsoid.x=grip
        return goal;

    def a_grasp_obj(self,grip):
        goal=SOMAFrameworkGoal();
        goal.controller=soma_ur5.msg.SOMAFrameworkGoal.FORCE;
        goal.max_duration=2.0
        goal.header.frame_id='base_link';
        goal.header.stamp=rospy.Time.now();
        goal.wrench.force.z=-5.0;
        goal.ellipsoid.x=grip;
        return goal

    def poses_callback(self,data):
        self.poses=data;
        #for i in range(0,len(self.poses.poses)):
        #    self.poses.poses[i]=self.add_pose_noise(self.poses.poses[i],0.10,0.01);
        self.got_poses=True;

    def add_pose_noise(self,pose,magr,magt):
        out_pose=Pose();
        Md=tt.euler_matrix(0,0,magr*np.random.randn());
        Md[0,3]=np.random.randn()*magt;
        Md[1,3]=np.random.randn()*magt;
        Md[2,3]=np.random.randn()*magt;

        Mi=tt.quaternion_matrix([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]);
        Mi[0,3]=pose.position.x;
        Mi[1,3]=pose.position.y;
        Mi[2,3]=pose.position.z;
        print(Md)
        Mr=tt.concatenate_matrices(Mi,Md);
        q=tt.quaternion_from_matrix(Mr);
        t=tt.translation_from_matrix(Mr);

        out_pose.position.x=t[0];
        out_pose.position.y=t[1];
        out_pose.position.z=t[2];
        out_pose.orientation.x=q[0];
        out_pose.orientation.y=q[1];
        out_pose.orientation.z=q[2];
        out_pose.orientation.w=q[3];
        return out_pose;



    def apply_ellipsoid(self,pose,ell):
        out_pose=Pose()
        Mir=tt.quaternion_matrix([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
        Mit=tt.translation_matrix([pose.position.x,pose.position.y,pose.position.z])
        Mi=tt.concatenate_matrices(Mit,Mir)
        Mr=tt.euler_matrix(0,math.pi/2,0);
        #Mr(:,4) => x=along smaller side of object y=along larger side of object z=up
        #Me (ellipsoid) =
        # center of the ellipsoid wrt the palm: 0.0262,   -0.0043,    0.0654
        # PGD wrt the palm: -0.0529    0.2892    0.9558

        if ell is True:
            #Me=tt.euler_matrix(0.3,0,0);
            Me=tt.euler_matrix(0.2938,0.0529, 0.0078)
            Me[0,3]=0.0282
            Me[1,3]=0.0187
            Me[2,3]=-0.0076
            #Me[0,3]=-0.0287
            #Me[1,3]=-0.0159
            #Me[2,3]=0.0112
            #Mr[0,3]=-0.03;
            #Mr[1,3]=-0.01;
        else:
            Me=tt.euler_matrix(0.0,0,0);
            Mr[0,3]=-0.03;
            Mr[1,3]=-0.01;

        #embed()
        Mt=tt.concatenate_matrices(Mi,Mr,Me);
        q=tt.quaternion_from_matrix(Mt);
        t=tt.translation_from_matrix(Mt);
        out_pose.position.x=t[0];
        out_pose.position.y=t[1];
        out_pose.position.z=t[2];
        out_pose.orientation.w=q[3];
        out_pose.orientation.x=q[0];
        out_pose.orientation.y=q[1];
        out_pose.orientation.z=q[2];
        return out_pose;

    def obj_to_goal_pose(self,pose,ellipsoid=True):
        in_pose=PoseStamped()
        in_pose.header=self.poses.header;
        in_pose.pose=pose;
        obj_pose_base=tf2_geometry_msgs.do_transform_pose(in_pose,self.bTc)
        obj_pose_base.pose=self.add_pose_noise(obj_pose_base.pose,0.0,0.0);
        out_pose=self.apply_ellipsoid(obj_pose_base.pose,ell=ellipsoid)
        debug_pose=PoseStamped()
        debug_pose.pose=out_pose;
        debug_pose.header.stamp=rospy.Time.now();
        debug_pose.header.frame_id='base_link';
        self.pub_debug.publish(debug_pose)
        return out_pose;

    def add_state(self,sm,name,action_goal,next_state):
        sm.StateMachine.add(name,smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction,goal=action_goal),transitions={'succeeded' : next_state,'preempted' : 'Failed','aborted' : next_state} );

    def run(self):
        while not rospy.is_shutdown():
            ellipsoid=True
            t=raw_input('Go? (press N to go without ellipsoid)')
            if(t=='N'):
                ellipsoid=False

            self.build_sm(self.poses,ellipsoid)

    def build_sm(self,poses,ellipsoid):
        rospy.loginfo("Ellipsoid "+str(ellipsoid))
        sm_top = smach.StateMachine(outcomes=['Success','Failed'])
        sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_START')
        sis.start()
        with sm_top:
            smach.StateMachine.add('Start', Initial(),transitions={'top_grasp':'TopStart0'});
            for i in range(0,len(poses.poses)):
                self.add_state(smach,'TopStart'+str(i),self.a_goto_initial(),'TopApproach'+str(i));
                #smach.StateMachine.add('WaitPo'+str(i), WaitPoses(),userdata=self.got_poses,transitions={'got_poses':'TopApproach'+str(i)});
                self.add_state(smach,'TopApproach'+str(i),self.a_goto_approach(poses.poses[i],ellipsoid),'TopReach'+str(i));
                self.add_state(smach,'TopReach'+str(i),self.a_approach_down(v=-0.02,t=10,f=3.0,grip=0),'TopAdjust'+str(i));
                self.add_state(smach,'TopAdjust'+str(i),self.a_approach_down(v=0.01,t=1,f=7.0,grip=0.0),'TopGrasp'+str(i));
                self.add_state(smach,'TopGrasp'+str(i),self.a_grasp_obj(grip=0.8),'TopLift'+str(i));
                self.add_state(smach,'TopLift'+str(i),self.a_approach_down(v=0.03,t=5,f=10.0,grip=0.8),'TopStart'+str(i+1));
                #smach.StateMachine.add('TopStart',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction,goal=self.goto_initial),transitions={'succeeded' : 'Success','preempted' : 'Failed','aborted' : 'Failed'} );
                #smach.StateMachine.add('TopApproach',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction,goal=self.goto_approch(poses.poses[i])),transitions={'succeeded' : 'Success','preempted' : 'Failed','aborted' : 'Failed'} );
                #smach.StateMachine.add('TopApproach',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction,goal=self.build_action()),transitions={'succeeded' : 'Success','preempted' : 'Failed','aborted' : 'Failed'} );
            self.add_state(smach,'TopStart'+str(i+1),self.a_goto_initial(),'Success'  );
        outcome = sm_top.execute()


def main():
    rospy.init_node('soma_iros_topgrasp_sm')
    tg=TopGraspFull()
    tg.run();


if __name__ == '__main__':
    main();
