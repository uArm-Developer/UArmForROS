#!/usr/bin/env python

'''

# File Name : move_to_node.py
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
from uarm.msg import Coords
from uarm.msg import CoordsWithTime
from uarm.msg import CoordsWithTS4

# raise error
def raiseError():
	print 'ERROR: Input 3 parameter represents x,y,z and time and servo_4 angle'

# main exection function
def execute():

	# define publisher and its topic 
	pub_1 = rospy.Publisher('move_to',Coords,queue_size = 10)
	pub_2 = rospy.Publisher('move_to_time',CoordsWithTime,queue_size = 10)
	pub_3 = rospy.Publisher('move_to_time_s4',CoordsWithTS4,queue_size = 10)
	rospy.init_node('move_to_node',anonymous = True)
	rate = rospy.Rate(10)

	# input x,y,z
	if len(sys.argv)>3:
		x = float(sys.argv[1])
		y = float(sys.argv[2])
		if y> 0:
			y = -y
		z = float(sys.argv[3])

	if len(sys.argv) == 4:
		pub_1.publish(x,y,z)
		
	elif len(sys.argv) == 5:
		Time = int(sys.argv[4])
		pub_2.publish(x,y,z,Time)

	elif len(sys.argv) == 6:
		Time = int(sys.argv[4])
		S4 = int(sys.argv[5])
		pub_3.publish(x,y,z,Time,S4)
	
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
	
	
