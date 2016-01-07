#!/usr/bin/env python

'''

# File Name : write_angles_node.py
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
from uarm.msg import Angles

# raise error
def raiseError():
	print 'ERROR: Input 4 servo angles between 0 ~ 180 degree'

# main exection function
def execute():

	# define publisher and its topic 
	pub = rospy.Publisher('write_angles',Angles,queue_size = 10)
	rospy.init_node('write_angles_node',anonymous = True)
	rate = rospy.Rate(10)
	
	# write 4 angles
	if len(sys.argv) == 5:
		s1 = int(sys.argv[1])
		s2 = int(sys.argv[2])
		s3 = int(sys.argv[3])
		s4 = int(sys.argv[4])
		pub.publish(s1,s2,s3,s4)
		
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
	
