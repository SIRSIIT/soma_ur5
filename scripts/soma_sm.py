#!/usr/bin/env python

#
# Copyright 2017 <copyright holder> <email>
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
#


import roslib; roslib.load_manifest('soma_ur5')
import rospy
import smach
import smach_ros
import actionlib
from soma_ur5.msg import *
import copy

class Waiting(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['top_grasp','slide_grasp','wall_grasp','none'],
			 input_keys=['object_pose','constraint_pose'],
			 output_keys=['object_pose','constraint_pose'])
    
  def execute(self, userdata):
    t=raw_input('What grasp?')
    if t == 'top':
      return 'top_grasp'
    elif t== 'slide':
      return 'slide_grasp'
    elif t== 'wall':
      return 'wall_grasp'
    else:
      return 'none'

class Done(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['restart'])    
  def execute(self, userdata):
    rospy.sleep(1)
    return 'restart';

  
  
class TopGraspStart(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['top_grasp','slide_grasp','wall_grasp','none'],
			 input_keys=['object_pose','constraint_pose'],
			 output_keys=['object_pose','constraint_pose'])
    
  def execute(self, userdata):
    rospy.sleep(1)    



def main():
    rospy.init_node('soma_state_machine')
    
    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['Failed'])
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_START')
    sis.start()
    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('Done', Done(),
			       transitions={'restart' : 'Start'})
			       
			       
        smach.StateMachine.add('Start', Waiting(), 
                               transitions={'top_grasp':'TopApproach', 
                                            'slide_grasp':'SlideApproach',
                                            'wall_grasp':'WallApproach',
                                            'none' : 'Done'})
    
	
	approach_goal=SOMAFrameworkGoal();	
	approach_goal.wrench.force.x=5.0;
	approach_goal.wrench.force.y=5.0;
	approach_goal.wrench.force.z=5.0;
	approach_goal.pose.position.x=0.24;
	approach_goal.pose.position.y=-0.625;
	approach_goal.pose.position.z=0.15;
	approach_goal.pose.orientation.w=0.39;
	approach_goal.pose.orientation.x=0.45;
	approach_goal.pose.orientation.y=0.59;
	approach_goal.pose.orientation.z=-0.53;
	approach_goal.max_duration=10;
	
	smach.StateMachine.add('TopApproach',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=approach_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'TopReach','preempted' : 'Failed','aborted' : 'Failed'} );   
		
	reach_goal=SOMAFrameworkGoal();
	reach_goal.wrench.force.x=4.0;
	reach_goal.wrench.force.y=4.0;
	reach_goal.wrench.force.z=-7.0;
	reach_goal.twist.linear.z=-0.01;
	reach_goal.max_duration=10;
	smach.StateMachine.add('TopReach',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=reach_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'TopGrasp','preempted' : 'Failed','aborted' : 'TopGrasp'} );   
	
	grasp_goal=SOMAFrameworkGoal();
	grasp_goal.wrench.force.z=-1.0;
	grasp_goal.ellipsoid.x=0.8;
	grasp_goal.max_duration=1;
	
	smach.StateMachine.add('TopGrasp',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=grasp_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'TopLift','preempted' : 'Failed','aborted' : 'TopLift'} );   
		
	lift_goal=SOMAFrameworkGoal();
	lift_goal.max_duration=4;
	lift_goal.twist.linear.z=0.02;
	#lift_goal.pose.position.z=0.3;
	lift_goal.ellipsoid.x=0.8;
	lift_goal.wrench.force.z=15.0;
	smach.StateMachine.add('TopLift',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=lift_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'Drop','preempted' : 'Failed','aborted' : 'Failed'} );   
	
	
	
	#SLIDE STATES
	s_approach_goal=SOMAFrameworkGoal();	
	s_approach_goal.wrench.force.x=5.0;
	s_approach_goal.wrench.force.y=5.0;
	s_approach_goal.wrench.force.z=7.0;	
	s_approach_goal.pose.position.x=-0.18;
	s_approach_goal.pose.position.y=-0.66;
	s_approach_goal.pose.position.z=0.15;		
	s_approach_goal.pose.orientation.w=0.36;
	s_approach_goal.pose.orientation.x=0.43;
	s_approach_goal.pose.orientation.y=0.63;
	s_approach_goal.pose.orientation.z=-0.52;
	s_approach_goal.max_duration=10;
	smach.StateMachine.add('SlideApproach',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=s_approach_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'SlideReach','preempted' : 'Failed','aborted' : 'Failed'} );   
		

	reach_goal.wrench.force.z=-11.0;
	
	smach.StateMachine.add('SlideReach',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=reach_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'SlideSlide','preempted' : 'Failed','aborted' : 'SlideSlide'} );   
	
	

	slide_goal=SOMAFrameworkGoal();
	slide_goal.max_duration=20;
	slide_goal.twist.linear.x=-0.02;
	slide_goal.twist.linear.z=-0.001;
	slide_goal.wrench.force.z=20;
	slide_goal.pose.position.x=-0.3;
	slide_goal.ellipsoid.x=0.0;
	
	smach.StateMachine.add('SlideSlide',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=slide_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'SlideGrasp','preempted' : 'Failed','aborted' : 'Failed'} );   
	
	
	s_grasp_goal=SOMAFrameworkGoal();
	s_grasp_goal.twist.linear.z=0.004;
	s_grasp_goal.wrench.force.z=20;
	s_grasp_goal.ellipsoid.x=0.8;
	s_grasp_goal.max_duration=2;


	smach.StateMachine.add('SlideGrasp',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=s_grasp_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'SlideLift','preempted' : 'Failed','aborted' : 'Failed'} );   

	smach.StateMachine.add('SlideLift',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=lift_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'Drop','preempted' : 'Failed','aborted' : 'Done'} );
	
	
	
	
	
	w_approach_goal=SOMAFrameworkGoal();
	w_approach_goal.pose.position.x=0.035
	w_approach_goal.pose.position.y=-0.60
	w_approach_goal.pose.position.z= 0.25
	w_approach_goal.pose.orientation.x= 0.50
	w_approach_goal.pose.orientation.y= 0.50
	w_approach_goal.pose.orientation.z= -0.50
	w_approach_goal.pose.orientation.w= 0.50
	w_approach_goal.wrench.force.z=-4.0
	w_approach_goal.max_duration=15;
	
	smach.StateMachine.add('WallApproach',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=w_approach_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'WallReach','preempted' : 'Failed','aborted' : 'Done'} );
	
	
	w_reach_goal=SOMAFrameworkGoal();
	w_reach_goal.max_duration=20;
	w_reach_goal.twist.angular.x=0.15;
	w_reach_goal.twist.linear.y=0.029;
	w_reach_goal.twist.linear.z=-0.021;
	w_reach_goal.wrench.force.y=17;
	w_reach_goal.wrench.force.z=18;
	w_reach_goal.ellipsoid.x=0.35;
	smach.StateMachine.add('WallReach',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=w_reach_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'WallLift','preempted' : 'Failed','aborted' : 'WallLift'} );
	


	w_lift_goal=SOMAFrameworkGoal();
	w_lift_goal.max_duration=6;
	w_lift_goal.twist.linear.y=-0.01;
	w_lift_goal.twist.linear.z=0.03;
	#lift_goal.pose.position.z=0.3;
	w_lift_goal.ellipsoid.x=0.8;
	w_lift_goal.wrench.force.z=25.0;
	smach.StateMachine.add('WallLift',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=w_lift_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'Drop','preempted' : 'Failed','aborted' : 'Failed'} );   
	


	w_drop_goal=SOMAFrameworkGoal();
	w_drop_goal.pose.position.x=0.03
	w_drop_goal.pose.position.y=-0.98
	w_drop_goal.pose.position.z= 0.35
	w_drop_goal.pose.orientation.x= 0.50
	w_drop_goal.pose.orientation.y= 0.50
	w_drop_goal.pose.orientation.z= -0.50
	w_drop_goal.pose.orientation.w= 0.50
	w_drop_goal.wrench.force.z=10.0
	w_drop_goal.ellipsoid.x=0.85;
	w_drop_goal.max_duration=10;
	smach.StateMachine.add('Drop',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=w_drop_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'Release','preempted' : 'Failed','aborted' : 'Failed'} );   
	
	w_release_goal=copy.deepcopy(w_drop_goal);
	w_release_goal.ellipsoid.x=0.0;
	smach.StateMachine.add('Release',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=w_release_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'Done','preempted' : 'Failed','aborted' : 'Failed'} );   
	




    # Execute SMACH plan
    outcome = sm_top.execute()


if __name__ == '__main__':
    main()

