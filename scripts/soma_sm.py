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
    sm = smach.StateMachine(outcomes=['Done', 'Failed'])
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_START')
    sis.start()
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'Joao', 
                                            'outcome2':'Joao'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome2':'Done'})
	act_goal=SOMAFrameworkGoal();	
	act_goal.wrench.force.z=10;
	act_goal.max_duration=5;
	smach.StateMachine.add('Joao',smach_ros.SimpleActionState('soma_action',SOMAFrameworkAction, \
							      goal=act_goal),
							   #goal_slots=['pose','twist', 'wrench']), 
							     transitions={'succeeded' : 'BAR','preempted' : 'FOO','aborted' : 'FOO'} );							   
	

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()

