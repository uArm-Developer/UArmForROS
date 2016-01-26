#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import sys
import math 
import time
from std_msgs.msg import String
from std_msgs.msg import Int32

PI = math.pi

def main_fcn():
	pub = rospy.Publisher('joint_states',JointState,queue_size = 10)
	pub2 = rospy.Publisher('read_angles',Int32,queue_size = 10)
	pub3 = rospy.Publisher('uarm_status',String,queue_size = 100)

	rospy.init_node('display',anonymous = True)
	rate = rospy.Rate(20)
	

	joint_state_send = JointState()
	joint_state_send.header = Header()
	joint_state_send.name = ['base_to_base_rot','base_rot_to_link_1','link_1_to_link_2' ,'link_2_to_link_3','link_3_to_link_end']


	joint_state_send.name = joint_state_send.name+ ['link_1_to_add_1','link_2_to_add_4','link_3_to_add_2','base_rot_to_add_3','link_2_to_add_5']
	
	angle = {}

	trigger = 1

	while not rospy.is_shutdown():
		joint_state_send.header.stamp = rospy.Time.now()

		pub2.publish(1)
		if trigger == 1:
			pub3.publish("detach")
			time.sleep(1)
			trigger = 0		
				
		angle['s1'] = rospy.get_param('servo_1')*math.pi/180
		angle['s2'] = rospy.get_param('servo_2')*math.pi/180
		angle['s3'] = rospy.get_param('servo_3')*math.pi/180
		angle['s4'] = rospy.get_param('servo_4')*math.pi/180

		joint_state_send.position = [angle['s1'],angle['s2'],-angle['s2']-angle['s3'],angle['s3'],angle['s4']]
		joint_state_send.position = joint_state_send.position + [-angle['s2']-angle['s3'],angle['s3'],PI/2-angle['s3']]
		joint_state_send.position = joint_state_send.position + [angle['s2']-PI,angle['s2']+angle['s3']-PI/2]
		joint_state_send.velocity = [0]
		joint_state_send.effort = []
		
		pub.publish(joint_state_send)
		rate.sleep()

if __name__ == '__main__':

	print 'begin'
	main_fcn()

