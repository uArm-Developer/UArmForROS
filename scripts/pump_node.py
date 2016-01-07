#!/usr/bin/env python

'''

# File Name : pump_node.py
# Author : Joey Song
# Version : V1.0
# Date : 6 Jan, 2016
# Modified Date : 6 Jan, 2016
# Description : This documents is for uarm ROS Library and ROS package
# Copyright(C) 2016 uArm Team. All right reserved.

'''

# All libraries needed to import 
# Import system library

import rospy
import sys
from std_msgs.msg import String

# raise error
def raiseError():
	print 'ERROR: Input Incorrect'
	print 'ERROR: Input off / low / 0  or  on / high / 1'

# main exection function
def execute():
	
	# define publisher and its topic 
	pub = rospy.Publisher('pump_str_control',String,queue_size = 10)
	rospy.init_node('pump_node',anonymous = True)
	rate = rospy.Rate(10)
	
	# process different inputs
	if len(sys.argv) == 2:
		data_input = sys.argv[1]
		
		# control pump on
		if data_input.lower() == 'on' or data_input == '1' or data_input.lower() == 'high':
			pub.publish('on')
		
		# control pump off
		elif data_input.lower() == 'off' or data_input == '0' or data_input.lower() == 'low':
			pub.publish('off')
		else:
			raiseError()
	else:
		raiseError()

	rate.sleep()


# main function
if __name__ == '__main__':
	try:
		execute()
	except:
		print '=========================================='
		print 'ERROR: exectuion error'
		raiseError()
		pass
