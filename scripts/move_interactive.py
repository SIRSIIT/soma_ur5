#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, PoseStamped
from tf.broadcaster import TransformBroadcaster
from qb_interface.msg import handRef

from random import random
from math import sin

class UR5_Interactive:
    server = None
    menu_handler = MenuHandler()
    br = None
    counter = 0

    
    def cb_once(this,msg):
        this.initial_pose=msg.pose;


    def frameCallback(this, msg ):
        time = rospy.Time.now()
        #br.sendTransform( (0, 0, sin(counter/140.0)*2.0), (0, 0, 0, 1.0), time, "base_link", "moving_frame" )
        pose=PoseStamped();

        pose.pose=this.int_marker.pose;
        pose.header=this.int_marker.header;
        this.goal_pub.publish(pose);
        this.counter += 1

    def processFeedback(this, feedback ):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo( s + ": button click" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )            
            hand_ref=handRef();            
            if feedback.menu_entry_id==1:
                hand_ref.closure.append(0.0)
                this.grasp_pub.publish(hand_ref)
            else:
                hand_ref.closure.append(1.0)
                this.grasp_pub.publish(hand_ref)
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo( s + ": pose changed")
    # TODO
    #          << "\nposition = "
    #          << feedback.pose.position.x
    #          << ", " << feedback.pose.position.y
    #          << ", " << feedback.pose.position.z
    #          << "\norientation = "
    #          << feedback.pose.orientation.w
    #          << ", " << feedback.pose.orientation.x
    #          << ", " << feedback.pose.orientation.y
    #          << ", " << feedback.pose.orientation.z
    #          << "\nframe: " << feedback.header.frame_id
    #          << " time: " << feedback.header.stamp.sec << "sec, "
    #          << feedback.header.stamp.nsec << " nsec" )
    #    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
    #        rospy.loginfo( s + ": mouse down" + mp + "." )
    #    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
    #        rospy.loginfo( s + ": mouse up" + mp + "." )
        this.server.applyChanges()


    def rand(this, min_, max_ ):
        return min_ + random()*(max_-min_)

    def makeBox(this, msg ):
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        return marker

    def makeBoxControl(this, msg ):
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( this.makeBox(msg) )
        msg.controls.append( control )
        return control

    def saveMarker(this, int_marker ):
      this.server.insert(int_marker, this.processFeedback)


    #####################################################################
    # Marker Creation

    def make6DofMarker(this, fixed, interaction_mode, pose, show_6dof = False):
        this.int_marker.header.frame_id = "base_link"
        this.int_marker.pose.position = pose.position
        this.int_marker.pose.orientation = pose.orientation
        this.int_marker.scale = 0.2

        this.int_marker.name = "simple_6dof"
        this.int_marker.description = "Simple 6-DOF Control"

        # insert a box
        this.makeBoxControl(this.int_marker)
        this.int_marker.controls[0].interaction_mode = interaction_mode

        if fixed:
            this.int_marker.name += "_fixed"
            this.int_marker.description += "\n(fixed orientation)"

        if interaction_mode != InteractiveMarkerControl.NONE:
            control_modes_dict = { 
                              InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                              InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                              InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
            this.int_marker.name += "_" + control_modes_dict[interaction_mode]
            this.int_marker.description = "3D Control"
            if show_6dof: 
              this.int_marker.description += " + 6-DOF controls"
            this.int_marker.description += "\n" + control_modes_dict[interaction_mode]
        
        if show_6dof: 
            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 1
            control.orientation.y = 0
            control.orientation.z = 0
            control.name = "rotate_x"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            this.int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 1
            control.orientation.y = 0
            control.orientation.z = 0
            control.name = "move_x"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            this.int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 1
            control.orientation.z = 0
            control.name = "rotate_z"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            this.int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 1
            control.orientation.z = 0
            control.name = "move_z"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            this.int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 0
            control.orientation.z = 1
            control.name = "rotate_y"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            this.int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 0
            control.orientation.z = 1
            control.name = "move_y"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            this.int_marker.controls.append(control)

            # Try to add a button
            #control = InteractiveMarkerControl()
            #control.interaction_mode = InteractiveMarkerControl.MENU
            #control.name = "menu"
            #control.description="Options"
            #this.int_marker.controls.append(copy.deepcopy(control))
                       


        this.server.insert(this.int_marker, this.processFeedback)
        this.menu_handler.apply( this.server, this.int_marker.name )

    def go(this):
        this.int_marker = InteractiveMarker()
        this.goal_pub = rospy.Publisher('goal_pose', geometry_msgs.msg.PoseStamped, queue_size=10)     
        #this.grasp_pub = rospy.Publisher('/soft_hand/joint_position_controller/command', std_msgs.msg.Float64, queue_size=10)     
        this.grasp_pub = rospy.Publisher('/qb_class/hand_ref', handRef, queue_size=10)     

        this.sub_once=None
        this.initial_pose=None  
        this.sub_once = rospy.Subscriber("ee_pose", geometry_msgs.msg.PoseStamped, this.cb_once)

        this.menu_handler = MenuHandler()
        this.menu_handler.insert( "Open Hand", callback=this.processFeedback )
        this.menu_handler.insert( "Close Hand", callback=this.processFeedback )



        while this.initial_pose==None:
            rospy.sleep(0.1)

        this.sub_once.unregister()
        rospy.Timer(rospy.Duration(0.01), this.frameCallback)
        this.server = InteractiveMarkerServer("ur5_interactive")

        this.make6DofMarker( False, InteractiveMarkerControl.NONE, this.initial_pose, True)
        this.server.applyChanges()

        while not rospy.is_shutdown():
            rospy.spin()


if __name__=="__main__":
    rospy.init_node("ur5_interactive_node")
    node=UR5_Interactive()
    # create a timer to update the published transforms
    node.go();

    

