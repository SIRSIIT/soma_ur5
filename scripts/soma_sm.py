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

class Waiting(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['top_grasp','slide_grasp','none'],
			 input_keys=['object_pose','constraint_pose'],
			 output_keys=['object_pose','constraint_pose'])
    
  def execute(self, userdata):
    t=raw_input('What grasp?')
    if t == 'top':
      return 'top_grasp'
    elif t== 'slide':
      return 'slide_grasp'
    else:
      return 'none'

class Done(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['done'])    
  def execute(self, userdata):
    rospy.sleep(1)
    return 'done';

  
  
class TopGraspStart(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['top_grasp','slide_grasp','none'],
			 input_keys=['object_pose','constraint_pose'],
			 output_keys=['object_pose','constraint_pose'])
    
  def execute(self, userdata):
    rospy.sleep(1)    




class Foo(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['outcome1','outcome2'])
    self.counter = 0

  def execute(self, userdata):
    rospy.loginfo('Executing state FOO')
    if self.counter < 1:
      rospy.sleep(1)
      self.counter += 1
      return 'outcome1'
    else:
      return 'outcome2'  
    
    
    
    
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.sleep(1)
        return 'outcome2'

class SOMA_SM:  
    def __init__(self):
        pass

def main():
    rospy.init_node('soma_state_machine')
    
    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['Done', 'Failed'])
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_START')
    sis.start()
    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('Start', Waiting(), 
                               transitions={'top_grasp':'TopApproach', 
                                            'slide_grasp':'SlideApproach',
                                            'none' : 'Done'})
    
	
	approach_goal=SOMAFrameworkGoal();	
	approach_goal.wrench.force.x=5.0;
	approach_goal.wrench.force.y=5.0;
	approach_goal.wrench.force.z=5.0;
	approach_goal.pose.position.x=0.01;
	approach_goal.pose.position.y=-0.625;
	approach_goal.pose.position.z=0.15;	
	approach_goal.pose.orientation.w=0.41;
	approach_goal.pose.orientation.x=0.41;
	approach_goal.pose.orientation.y=0.57;
	approach_goal.pose.orientation.z=-0.57;
	approach_goal.max_duration=10;
	
	smach.StateMachine.add('TopApproach',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=approach_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'TopReach','preempted' : 'Failed','aborted' : 'Failed'} );   
		
	reach_goal=SOMAFrameworkGoal();
	reach_goal.wrench.force.x=4.0;
	reach_goal.wrench.force.y=4.0;
	reach_goal.wrench.force.z=-7.0;
	reach_goal.twist.linear.z=-0.03;
	reach_goal.max_duration=10;
	smach.StateMachine.add('TopReach',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=reach_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'TopGrasp','preempted' : 'Failed','aborted' : 'TopGrasp'} );   
	
	grasp_goal=SOMAFrameworkGoal();
	grasp_goal.wrench.force.z=-4.0;
	grasp_goal.ellipsoid.x=0.8;
	grasp_goal.max_duration=3;
	
	smach.StateMachine.add('TopGrasp',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=grasp_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'TopLift','preempted' : 'Failed','aborted' : 'TopLift'} );   
		
	lift_goal=SOMAFrameworkGoal();
	lift_goal.max_duration=3;
	lift_goal.twist.linear.z=0.02;
	#lift_goal.pose.position.z=0.3;
	lift_goal.ellipsoid.x=0.7;
	lift_goal.wrench.force.z=15.0;
	smach.StateMachine.add('TopLift',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=lift_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'Done','preempted' : 'Failed','aborted' : 'Failed'} );   
	
	
	
	#SLIDE STATES
	approach_goal.pose.orientation.w=0.37;
	approach_goal.pose.orientation.x=0.40;
	approach_goal.pose.orientation.y=0.60;
	approach_goal.pose.orientation.z=-0.57;
	smach.StateMachine.add('SlideApproach',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=approach_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'SlideReach','preempted' : 'Failed','aborted' : 'Failed'} );   
		

	reach_goal.wrench.force.z=-8.0;
	
	smach.StateMachine.add('SlideReach',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=reach_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'SlideSlide','preempted' : 'Failed','aborted' : 'SlideSlide'} );   
	
	

	slide_goal=SOMAFrameworkGoal();
	slide_goal.max_duration=10;
	slide_goal.twist.linear.x=-0.03;
	slide_goal.wrench.force.z=10;
	slide_goal.pose.position.x=-0.35;
	slide_goal.ellipsoid.x=0.1;
	
	smach.StateMachine.add('SlideSlide',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=slide_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'SlideGrasp','preempted' : 'Failed','aborted' : 'SlideGrasp'} );   
	
	

	smach.StateMachine.add('SlideGrasp',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=grasp_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'SlideLift','preempted' : 'Failed','aborted' : 'SlideLift'} );   

	smach.StateMachine.add('SlideLift',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=lift_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'Done','preempted' : 'Failed','aborted' : 'Done'} );   




    # Execute SMACH plan
    outcome = sm_top.execute()


if __name__ == '__main__':
    main()

