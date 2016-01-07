#!/usr/bin/env python

'''

# File Name : report_coords_node.py
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
import time
from std_msgs.msg import Int32

# raise error
def raiseError():
	print 'ERROR: Input Incorrect'
	print 'ERROR: 3 options for print coords'
	print 'ERROR: 1: Input nothing after report_coords_node.py (example: rosrun uarm report_coords_node.py)'
	print 'ERROR: 2: Input how many times to print (example: rosrun uarm report_coords_node.py 12)'
	print 'ERROR: 3: Input times and time_sec between each times (example: rosrun uarm report_coords_node.py 12 2)'

# main exection function
def execute():

	# define publisher and its topic 
	pub = rospy.Publisher('read_coords',Int32,queue_size = 10)
	rospy.init_node('report_coords_node',anonymous = True)
	rate = rospy.Rate(10)
	
	# report once
	if len(sys.argv) == 1:
		pub.publish(1)
		time.sleep(0.1)
		print 'Current location is x: %2.2fcm, y: %2.2fcm, z: %2.2fcm.' %(rospy.get_param('current_x'),rospy.get_param('current_y'),rospy.get_param('current_z'))

	# report multiple time
	elif len(sys.argv) == 2:
		for i in range(0,int(sys.argv[1])):
			pub.publish(1)
			time.sleep(0.1)
			print 'Current location is x: %2.2fcm, y: %2.2fcm, z: %2.2fcm.' %(rospy.get_param('current_x'),rospy.get_param('current_y'),rospy.get_param('current_z'))

	elif len(sys.argv) == 3:
		for i in range(0,int(sys.argv[1])):
			pub.publish(1)
			time.sleep(0.1)
			print 'Current location is x: %2.2fcm, y: %2.2fcm, z: %2.2fcm.' %(rospy.get_param('current_x'),rospy.get_param('current_y'),rospy.get_param('current_z'))
			time.sleep(float(sys.argv[2])-0.1)
	else:
		raiseError()

	rate.sleep()
			
# main function
if __name__ == '__main__':
	try:
		execute()
		pass
	except:
		print '=========================================='
		print 'ERROR: Exectuion error. Input numbers only'
		raiseError()
		pass
