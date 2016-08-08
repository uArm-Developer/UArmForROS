#!/usr/bin/env python
import rospy
import copy
import math
import sys

from sensor_msgs.msg import JointState

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32

from math import sin
import time
from uarm.msg import Angles
from uarm.msg import CoordsWithTime

server = None
menu_handler = MenuHandler()
br = None
counter = 0

PI = math.pi
MATH_PI	= 3.141592653589793238463
MATH_TRANS  = 57.2958
MATH_L1	= (10.645+0.6)
MATH_L2	= 2.117
MATH_L3	= 14.825
MATH_L4	= 16.02
MATH_L43 = MATH_L4/MATH_L3

# UARM OFFSETS
TopOffset = -1.5
BottomOffset = 1.5

angle = {}

def ivsKine(x, y, z):
		if z > (MATH_L1 + MATH_L3 + TopOffset):
			z = MATH_L1 + MATH_L3 + TopOffset
		if z < (MATH_L1 - MATH_L4 + BottomOffset):
			z = MATH_L1 - MATH_L4 + BottomOffset

		g_y_in = (-y-MATH_L2)/MATH_L3
		g_z_in = (z - MATH_L1) / MATH_L3
		g_right_all = (1 - g_y_in*g_y_in - g_z_in*g_z_in - MATH_L43*MATH_L43) / (2 * MATH_L43)
		g_sqrt_z_y = math.sqrt(g_z_in*g_z_in + g_y_in*g_y_in)

		if x == 0:
			# Calculate value of theta 1
			g_theta_1 = 90;
			# Calculate value of theta 3
			if g_z_in == 0:
				g_phi = 90
			else:
				g_phi = math.atan(-g_y_in / g_z_in)*MATH_TRANS
			if g_phi > 0:
				g_phi = g_phi - 180
	    		g_theta_3 = math.asin(g_right_all / g_sqrt_z_y)*MATH_TRANS - g_phi

	    		if g_theta_3<0:
				g_theta_3 = 0
			# Calculate value of theta 2
	    		g_theta_2 = math.asin((z + MATH_L4*math.sin(g_theta_3 / MATH_TRANS) - MATH_L1) / MATH_L3)*MATH_TRANS
		else:
			# Calculate value of theta 1
			g_theta_1 = math.atan(y / x)*MATH_TRANS
			if (y/x) > 0:
				g_theta_1 = g_theta_1
			if (y/x) < 0:
				g_theta_1 = g_theta_1 + 180
			if y == 0:
				if x > 0:
					g_theta_1 = 180
				else: 
					g_theta_1 = 0
			# Calculate value of theta 3
			g_x_in = (-x / math.cos(g_theta_1 / MATH_TRANS) - MATH_L2) / MATH_L3;
			if g_z_in == 0:  
				g_phi = 90
			else:
				g_phi = math.atan(-g_x_in / g_z_in)*MATH_TRANS
			if g_phi > 0:
				g_phi = g_phi - 180 

			g_sqrt_z_x = math.sqrt(g_z_in*g_z_in + g_x_in*g_x_in)

			g_right_all_2 = -1 * (g_z_in*g_z_in + g_x_in*g_x_in + MATH_L43*MATH_L43 - 1) / (2 * MATH_L43)
			g_theta_3 = math.asin(g_right_all_2 / g_sqrt_z_x)*MATH_TRANS
			g_theta_3 = g_theta_3 - g_phi

			if g_theta_3 <0 :
				g_theta_3 = 0
			# Calculate value of theta 2
			g_theta_2 = math.asin(g_z_in + MATH_L43*math.sin(abs(g_theta_3 / MATH_TRANS)))*MATH_TRANS

		g_theta_1 = abs(g_theta_1);
		g_theta_2 = abs(g_theta_2);

		angle[1] = g_theta_1
		angle[2] = g_theta_2
		angle[3] = g_theta_3
		angle[4] = 0
		return angle



def processFeedback( feedback ):
    
    current_x = -round(feedback.pose.position.x*1000)/10.0
    current_y = -round(feedback.pose.position.y*1000)/10.0
    current_z = round(feedback.pose.position.z*1000)/10.0

    print 'x: ' + str(current_x) +'cm y: ' + str(current_y) + 'cm z: '+str(current_z) +'cm'
    display(ivsKine(current_x,current_y,current_z))

    pub3.publish(current_x,current_y,current_z,0)

    server.applyChanges()

def display(angle):
    for i in angle:
        angle[i] = angle[i]*math.pi/180
    joint_state_send.header.stamp = rospy.Time.now()
    joint_state_send.position = [angle[1],angle[2],-angle[2]-angle[3],angle[3],angle[4]]
    joint_state_send.position = joint_state_send.position + [-angle[2]-angle[3],angle[3],PI/2-angle[3]]
    joint_state_send.position = joint_state_send.position + [angle[2]-PI,angle[2]+angle[3]-PI/2]
    joint_state_send.velocity = [0]
    joint_state_send.effort = []
    pub.publish(joint_state_send)

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.SPHERE


    marker.scale.x = msg.scale * 0.1
    marker.scale.y = msg.scale * 0.1
    marker.scale.z = msg.scale * 0.1
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 0.1

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

#####################################################################
# Marker Creation

def make6DofMarker( fixed, interaction_mode, position, show_6dof = False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base"
    int_marker.pose.position = position	# Defined the position of the marker
    int_marker.scale = 0.1

    int_marker.name = "simple_6dof"
    int_marker.description = "uarm_controller"

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if show_6dof: 
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
      
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )

if __name__=="__main__":

    rospy.init_node("visual_control")
    pub = rospy.Publisher('joint_states',JointState,queue_size = 10)
    pub2 = rospy.Publisher('read_coords',Int32,queue_size = 10)	
    pub3 = rospy.Publisher('move_to_time',CoordsWithTime,queue_size = 100)
    time.sleep(1)
    pub2.publish(1)
    time.sleep(1)
    

    joint_state_send = JointState()
    joint_state_send.header = Header()
    joint_state_send.name = ['base_to_base_rot','base_rot_to_link_1','link_1_to_link_2' ,'link_2_to_link_3','link_3_to_link_end']
    joint_state_send.name = joint_state_send.name+ ['link_1_to_add_1','link_2_to_add_4','link_3_to_add_2','base_rot_to_add_3','link_2_to_add_5']


    br = TransformBroadcaster()
    
    # create a timer to update the published transforms

    server = InteractiveMarkerServer("uarm_controller")

    menu_handler.insert( "First Entry", callback=processFeedback )
    menu_handler.insert( "Second Entry", callback=processFeedback )

    ini_x = -rospy.get_param('current_x')/100
    ini_y = -rospy.get_param('current_y')/100
    ini_z = rospy.get_param('current_z')/100
    time.sleep(1)

    position = Point( float(ini_x), float(ini_y), float(ini_z))
    display(ivsKine(float(ini_x),float(ini_y),float(ini_z)))
    #position = Point( 0,0 ,0 )
    make6DofMarker( True, InteractiveMarkerControl.NONE, position, True)
    server.applyChanges()

    rospy.spin()


