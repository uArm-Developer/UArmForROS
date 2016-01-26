#!/usr/bin/env python

'''

# File Name : uarm_stauts_node.py
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
	print 'ERROR: Input detach / 0 or attach / 1'

# main exection function
def execute():

	# define publisher and its topic 
	pub = rospy.Publisher('uarm_status',String,queue_size = 10)
	rospy.init_node('uarm_status_node',anonymous = True)
	rate = rospy.Rate(10)
	
	if len(sys.argv) == 2:
		data_input = sys.argv[1]

		# attach uarm
		if data_input.lower() == 'attach' or data_input == '1' :
			pub.publish('attach')

		# detach uarm
		elif data_input.lower() == 'detach' or data_input == '0' :
			pub.publish('detach')

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
