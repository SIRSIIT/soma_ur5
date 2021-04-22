#!/usr/bin/env python

import roslib; roslib.load_manifest('soma_ur5')
import rospy
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, TwistStamped, WrenchStamped
import tf2_ros
import tf2_geometry_msgs
from IPython import embed
import math
from tf import transformations as tt
import numpy as np
from std_srvs.srv import Trigger
from std_msgs.msg import String
import moveit_commander
import moveit_msgs.msg
import sys


class UR5MoveScoop:
    def __init__(self):       
        self.base_link='base_link'
        self.rate = rospy.Rate(50.0)
        #self.bias_srv = rospy.ServiceProxy('/arm_motion/bias_ft_sensor', Trigger)
        self.sub_ee_pose=rospy.Subscriber("/ur5/ee_pose",PoseStamped,self.ee_pose_cb, queue_size=2)
        if not self.wait_for_msg("/ur5/ee_pose"):
            exit(0)
        #self.sub_force=rospy.Subscriber("/ur5/ee_force_blink", WrenchStamped, self.force_callback)
        #if not self.wait_for_msg("/ur5/ee_force_blink"):
        #    exit(0)
        

        #self.scoop = Xbee()
        
        self.pub_pose=rospy.Publisher("/ur5/goal_pose", PoseStamped,queue_size=2)
        self.pub_gripper = rospy.Publisher('/cmd_gripper', String, queue_size=1)
        

        self.pub_traj_disp = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=2)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")

        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(120.0))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.paletta_pose=Pose()
        
        #aggiungere scaling velocity e accelerazioni
        #self.group.set_planner_id(self,"RRTstar")

    def wait_for_msg(self,topic_name):
        try:
            rospy.wait_for_message(topic_name, PoseStamped,1)
            return(True)
        except rospy.ROSException, e:
            rospy.logerr("Message timeout: call failed: %s", e)
            return(False)


    def ee_pose_cb(self,data):
        self.ee_pose=data

    '''
    def force_callback(self, msg):
        self.cur_force = msg

    def bias_sensor(self):
        try:
            resp = self.bias_srv()
            print(resp.message)
            return resp.success
        except rospy.ServiceException, e:
            print "Service call failegoal_pose=PoseStamped()d: %s" % e    
    '''
    def UR5GotoPlan(self,pose):
        goal=PoseStamped()
        goal.header.frame_id=self.base_link
        goal.header.stamp=rospy.Time.now()
        goal.pose=pose
        #(plan,fraction)=self.create_moveit_plan(goal)
        (plan)=self.create_crazy_moveit_plan(goal)
        if (plan.joint_trajectory.points):
            #t='R'
            #while(t == 'R'):
            t=raw_input('Go? (press y)')
            if(t == 'y'):
                self.group.execute(plan, wait=True)
                return True
            elif (t == 'R'):
                #embed()
                self.create_crazy_moveit_plan(goal)
                pass
            if(t == 'e'):
                embed()
            else:
                return False
        else:
            return False
    
    def UR5GotoJoint(self, joint,flag):
        self.group.clear_pose_targets()
        self.group.set_joint_value_target(joint)
        plan = self.group.plan()
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)        
        self.pub_traj_disp.publish(display_trajectory)
        if flag==0:
            t = raw_input("Go? (press y)")
            if(t=="y"):
                self.group.execute(plan, wait=True)
                return True
            else:
                return False
        else:
            self.group.execute(plan, wait=True)

    def create_moveit_plan(self,goal):
        #embed()
        self.group.clear_pose_targets()
        #self.group.set_pose_target(goal)
        #plan1 = self.group.plan()
        poses=[]
        poses.append(self.ee_pose.pose)
        poses.append(goal.pose)
        (plan, fraction) = self.group.compute_cartesian_path(poses,0.005,1)
        rospy.loginfo("fraction: %f",fraction)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)        
        self.pub_traj_disp.publish(display_trajectory);
        return (plan,fraction)

    def create_crazy_moveit_plan(self,goal):
        self.group.clear_pose_targets()
        self.group.set_pose_target(goal)    
        plan = self.group.plan()
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)        
        self.pub_traj_disp.publish(display_trajectory)
        return plan

    def grasp_object(self,command):    
        self.pub_gripper.publish(command)
        return True

    def pre_grasp_pose(self,obj_pose,table_pose,offset,gamma):
        
        new_pose = deepcopy(obj_pose)
        
        # Kinect2World :  rosrun tf tf_echo world kinect2_link
        #t = tt.translation_matrix([0.455, 1.293, 0.476])
        #R = tt.quaternion_matrix([-0.020, 0.733, -0.680, 0.012])
        #t = tt.translation_matrix([0.425, 1.233, 0.436])
        R = tt.quaternion_matrix([-0.007, 0.727, -0.687, 0.015])
        t = tt.translation_matrix([0.42, 1.29, 0.451])
        T = tt.concatenate_matrices(t,R)

        R_table = tt.quaternion_matrix([table_pose.pose.orientation.x, table_pose.pose.orientation.y,\
                                        table_pose.pose.orientation.z, table_pose.pose.orientation.w])
        
        R_tableW = T.dot(R_table) #Table in World Coordinate Frame
        Rz180 = tt.euler_matrix(0,0,math.pi)
        R1 = R_tableW.dot(Rz180)  #Additional rotation of 180 around z
        
        Rx = tt.euler_matrix(-math.pi/2,0,0)  
        R_goal = R1.dot(Rx)

        #Rx_temp = tt.euler_matrix(math.pi/2,0,0)## USE CASE OBJINTHEBOX
        #R_goal = R_goal.dot(Rx_temp)## USE CASE OBJINTHEBOX
        #Add gamma 
        Ry=tt.euler_matrix(0,gamma,0); 
        R_goal = R_goal.dot(Ry) #Rotation around y (Current Frame)
        
        offset_world = R_goal.dot([offset[0], offset[1], offset[2], 0]) #Offset paletta_frame 2 World_frame
        
        new_pose_quaternion = tt.quaternion_from_matrix(R_goal)
        

        new_pose.position.x = obj_pose.position.x + offset_world[0]
        new_pose.position.y = obj_pose.position.y + offset_world[1]
        new_pose.position.z = obj_pose.position.z + offset_world[2]

        new_pose.orientation.x = new_pose_quaternion[0]
        new_pose.orientation.y = new_pose_quaternion[1]
        new_pose.orientation.z = new_pose_quaternion[2]
        new_pose.orientation.w = new_pose_quaternion[3]

        '''
        goal_pose=PoseStamped()
        goal_pose.header.frame_id='world'
        goal_pose.header.stamp=rospy.Time.now()
        goal_pose.pose = new_pose
        self.pub_newpose.publish(goal_pose)
        '''
        return new_pose