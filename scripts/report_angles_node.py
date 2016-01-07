#!/usr/bin/env python

'''

# File Name : report_anlges_node.py
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
	print 'ERROR: 3 options for print angles'
	print 'ERROR: 1: Input nothing after report_anlges_node.py (example: rosrun uarm report_angles_node.py)'
	print 'ERROR: 2: Input how many times to print (example: rosrun uarm report_angles_node.py 12)'
	print 'ERROR: 3: Input times and time_sec between each times (example: rosrun uarm report_angles_node.py 12 2)'

# main exection function
def execute():

	# define publisher and its topic 
	pub = rospy.Publisher('read_angles',Int32,queue_size = 10)
	rospy.init_node('report_angles_node',anonymous = True)
	rate = rospy.Rate(10)

	# report once		
	if len(sys.argv) == 1:
		pub.publish(1)
		time.sleep(0.15)
		print 'Four Servo Angles: %2.2f, %2.2f, %2.2f and %2.2f degrees.' %(rospy.get_param('servo_1'),rospy.get_param('servo_2'),rospy.get_param('servo_3'),rospy.get_param('servo_4'))

	# report multiple time
	elif len(sys.argv) == 2:
		for i in range(0,int(sys.argv[1])):
			pub.publish(1)
			time.sleep(0.15)
			print 'Four Servo Angles: %2.2f, %2.2f, %2.2f and %2.2f degrees.' %(rospy.get_param('servo_1'),rospy.get_param('servo_2'),rospy.get_param('servo_3'),rospy.get_param('servo_4'))

	elif len(sys.argv) == 3:
		for i in range(0,int(sys.argv[1])):
			pub.publish(1)
			time.sleep(0.15)
			print 'Four Servo Angles: %2.2f, %2.2f, %2.2f and %2.2f degrees.' %(rospy.get_param('servo_1'),rospy.get_param('servo_2'),rospy.get_param('servo_3'),rospy.get_param('servo_4'))
			time.sleep(float(sys.argv[2])-0.15)
	
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
