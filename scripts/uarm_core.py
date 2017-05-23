#!/usr/bin/env python

'''

# File Name : uarm_core.py
# Author : Joey Song
# Version : V1.0
# Date : 6 Jan, 2016
# Modified Date : 6 Jan, 2016
# Description : This documents is for uarm ROS Library and ROS package
# Copyright(C) 2016 uArm Team. All right reserved.

'''


# All libraries needed to import
# Import system library
import sys
import time
import rospy

# Import uarm for python library
#from UArmForPython.uarm_python import Uarm
import pyuarm

# Import messages type
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from std_msgs.msg import Int32
from uarm.msg import Angles
from uarm.msg import Coords
from uarm.msg import CoordsWithTime
from uarm.msg import CoordsWithTS4

# Read current Coords function
def readCurrentCoords():
	cc = uarm.get_position()
	print 'Current location is x: %2.2fcm, y: %2.2fcm, z: %2.2fcm.' %(float(cc[0]), float(cc[1]), float(cc[2]))

	rospy.set_param('current_x', cc[0])
	rospy.set_param('current_y', float(cc[1]))
	rospy.set_param('current_z', float(cc[2]))

# Read current Angles function
def readCurrentAngles():
	ra = {}
	ra['s1'] = uarm.get_servo_angle(0)
	ra['s2'] = uarm.get_servo_angle(1)
	ra['s3'] = uarm.get_servo_angle(2)
	ra['s4'] = uarm.get_servo_angle(3)

	print 'Four Servo Angles: %2.2f, %2.2f, %2.2f and %2.2f degrees.' %(ra['s1'], ra['s2'],ra['s3'],ra['s4'])

	rospy.set_param('servo_1',ra['s1'])
	rospy.set_param('servo_2',ra['s2'])
	rospy.set_param('servo_3',ra['s3'])
	rospy.set_param('servo_4',ra['s4'])

# Read stopper function
def readStopperStatus():
	val = uarm.get_tip_sensor()
	if val == True:
		print 'Stopper is actived'
		rospy.set_param('stopper_status','HIGH')
	else:
		print 'Stopper is not actived'
		rospy.set_param('stopper_status','LOW')


# Write single angle
def writeSingleAngle(num, angle):
	# limits of each servo
	s3_max = 170 - uarm.get_servo_angle(1)
	if num == 1:
		if angle > 160:
			angle = 160
		elif angle < 20:
			angle = 20
	elif num == 2:
		if angle > 140:
			angle = 140
		elif angle < 0:
			angle = 0
	elif num == 3:
		if angle > s3_max:
			angle = s3_max
		elif angle < 0:
			angle = 0
	elif num == 4:
		if angle > 180:
			angle = 180
		elif angle < 0:
			angle = 0
	else:
		print "Input incorrect: wrong servo number"

	uarm.set_servo_angle(num-1, angle)


# Write coordinate: x y z and speed
def writeCoordinate(coordinate):
	x = float(coordinate[1])
	y = float(coordinate[2])
	z = float(coordinate[3])
	if len(coordinate) == 4:
		speed = 5

	elif len(coordinate) == 5:
		speed = float(coordinate[4])

	else:
		print "Input incorrect: wrong number of arguments"

	if y > 0:
		y = -y

	uarm.set_position(x, y, z, speed)


# Main connect function
def connectFcn():

	global failed_number
	global connectionStatus
	connectionStatus = 0
	global uarm
	global controlFcnLoop
	global listenerFcn
	controlFcnLoop = True
	listenerFcn = True

	if len(sys.argv)<2:
		# 1 means no input argument
		failed_number = 20
		controlFcnLoop = False
		listenerFcn = False
		print 'Input Incorrect'
		sys.exit(0)
		return 1

	if sys.argv[1] == 'connect':
		print '======================================================='
		print 'Connecting ...'
		print '======================================================='

		if len(sys.argv) == 2:
			failed_number = 21
			uarm = pyuarm.get_uarm()
			uarm.connect()
			connectionStatus = 1

			print 'Connected'
			return 21

		elif len(sys.argv) == 3:

			failed_number = 22

			if sys.argv[2] == 'l':
				failed_number = 23
				uarm = pyuarm.get_uarm()
				uarm.connect()
				controlFcnLoop = False
				print 'Connected'
			else:
				connectionStatus = 1
				print 'Connected'
			return 22

		# followed by l - directly listen to nodes
		elif len(sys.argv) == 4:
			failed_number = 23
			uarm =  pyuarm.get_uarm()
			uarm.connect()

			connectionStatus = 1
			print 'Connected'

			if sys.argv[3] == 'l':
				print 'Directly to listen mode'
				controlFcnLoop = False
				return 25
			return 23

		else:
			# 2 means input argument is wrong
			return 24
	#if sys.argv[1] == 'd'


# Main control function
def controlFcn():

	while controlFcnLoop:
		commands = raw_input("Input Commands (Input h to see all commands): ")

		if commands == 'h':
			print ' '
			print 'Control mode'
			print '=================================================================='
			print 'h                  - show help and list all commands'
			print 'cc                 - current coordinates'
			print 'ra                 - read angles'
			print 'wa a1 a2 a3 a4     - write angles'
			print '                     for example: wa 120 80 40 40'
			print 'wsa servo a        - write single angle'
			print '                     for example: wsa 1 50'
			print 'mt x y z           - move to a point with speed(defaut: 5mm/s)'
			print '                     for example: mt 0 12 12'
			print 'mt x y z speed     - move to a point with speed(mm/s)'
			print '                     for example: mt 0 12 12 10'
			print 'pp 1/0             - pump on/off'
			print 'ss                 - status of stopper'
			print 'at                 - attach uarm'
			print 'de                 - detach uarm'
			print 'l                  - begin listener mode'
			print 'e                  - exit'
			print '=================================================================='
			print ' '

		elif commands == 'l':
			print 'Exit: Break the control fuction loop'
			break;

		elif commands == 'e':
			print 'Detach all servos and exit the program'
			uarm.set_servo_detach()
			sys.exit(0)

		elif len(commands) == 0:
			print 'len is 0'

		else:
			commands_split = commands.split()

			# Detach
			if commands_split[0] == 'detach'or commands_split[0] == 'de':
				if len(commands_split) == 1:
					uarm.set_servo_detach()
				else:
					print 'no other commands should be input'
				pass

			# Attach
			if commands_split[0] == 'attach'or commands_split[0] == 'at':
				if len(commands_split) == 1:
					uarm.set_servo_attach()
				else:
					print 'no other commands should be input'
				pass

			# Stopper Status
			elif commands_split[0] == 'stopperStatus'or commands_split[0] == 'ss':
				if len(commands_split) == 1:
					readStopperStatus()
				else:
					print 'no other commands should be input'
				pass

			# Pump Control
			elif commands_split[0] == 'pump' or commands_split[0] == 'pp':
				if len(commands_split) == 1:
					print 'Status should be input'
				elif len(commands_split) >2:
					print 'Too many inputs'
				else:
					if commands_split[1] == '1' or commands_split[1].lower() == 'high' or commands_split[1].lower() == 'on':
						uarm.set_pump(1)
					elif commands_split[1] == '0' or commands_split[1].lower() == 'low'or commands_split[1].lower() == 'off':
						uarm.set_pump(0)
					else:
						print 'Incorrect inputs, should input 1 / 0 / HIGH / LOW / ON / OFF'
				pass

			# Current Coordinates
			elif commands_split[0] == 'currentCoords' or commands_split[0] == 'cc':
				if len(commands_split) == 1:
					readCurrentCoords()
				else:
					print 'no other commands should be input'
				pass

			# Write Single Angles
			elif commands_split[0] == 'writeSingleAngles' or commands_split[0] == 'wsa':
				if len(commands_split) == 3:
					writeSingleAngle(int(commands_split[1]), float(commands_split[2]))
				else:
					print 'Input incorrect: Wrong number of arguments'
				pass

			# Write Angles
			elif commands_split[0] == 'wa':
				if len(commands_split) == 5:
					writeSingleAngle(1, float(commands_split[1]))
					writeSingleAngle(2, float(commands_split[2]))
					writeSingleAngle(3, float(commands_split[3]))
					writeSingleAngle(4, float(commands_split[4]))
				else:
					print 'Input incorrect: Wrong number of arguments'
				pass

			# Current Angles
			elif commands_split[0] == 'readAngles' or commands_split[0] == 'ra':
				if len(commands_split) == 1:
					 readCurrentAngles()
				else:
					print 'no other commands should be input'
				pass

			# Move Tos
			elif commands_split[0] == 'moveTo' or commands_split[0] == 'mt':
				if len(commands_split) == 4 or len(commands_split) == 5:
					writeCoordinate(commands_split)
				else:
					print 'Input incorrect: Wrong number of arguments'
			else:
				pass


# pump control function once received data from topic
def pumpCallack(data):

	data_input = data.data

	if data_input == 0:
		uarm.pump_control(0)
		print 'Pump: Off'
	elif data_input == 1:
		uarm.pump_control(1)
		print 'Pump: On'
	else:
		pass


# pump str control function once received data from topic
def pumpStrCallack(data):

	data_input = data.data
	print data_input

	if data_input.lower() == 'low' or data_input.lower() == 'off':
		uarm.pump_control(0)
		print 'Pump: Off'
	elif data_input.lower() == 'on' or data_input.lower() == 'high':
		uarm.pump_control(1)
		print 'Pump: On'
	else:
		pass


# angles control function once received data from topic
def writeAnglesCallback(servos):

	servo = {}
	servo['s1'] = servos.servo_1
	servo['s2'] = servos.servo_2
	servo['s3'] = servos.servo_3
	servo['s4'] = servos.servo_4
	for i in servo:
		if servo[i]>180: servo[i] = 180
		if servo[i]<0: servo[i] = 0

	uarm.set_servo_angle(0, servo['s1'])
	uarm.set_servo_angle(1, servo['s2'])
	uarm.set_servo_angle(2, servo['s3'])
	uarm.set_servo_angle(3, servo['s4'])

	print 'Movement: Moved Once'


# attach or detach uarm function once received data from topic
def attchCallback(attachStatus):
	data_input = attachStatus.data

	if data_input.lower() == 'attach' :
		uarm.attach_all_servos()
		print 'uArm: Attach'
	elif data_input.lower() == 'detach':
		uarm.detach_all_servos()
		print 'uArm: Detach'
	else:
		pass


# move to function once received data from topic
def moveToCallback(coords):
	x = coords.x
	y = coords.y
	if y>0:
		y = -y
	z = coords.z
	uarm.set_position(x, y, z, 2)
	print 'Movement: Moved Once'


# moveto functions once received data from topic
def moveToTimeCallback(coordsAndT):
	x = coordsAndT.x
	y = coordsAndT.y
	if y>0:
		y = -y
	z = coordsAndT.z
	time = coordsAndT.time
	if time == 0:
		uarm.set_position(x, y, z, 2)
	else:
		uarm.set_position(x, y, z, 2)

	print 'Movement: Moved Once'
	pass


# moveto functions once received data from topic
def moveToTimeAndS4Callback(coordsAndTS4):

	x = coordsAndTS4.x
	y = coordsAndTS4.y
	if y>0:
		y = -y
	z = coordsAndTS4.z
	time = coordsAndTS4.time
	s4 = coordsAndTS4.servo_4
	if s4 > 180: s4 = 180
	if s4 <0 : s4 =0
	uarm.set_position(x, y, z, s4, 0, time, 0, 0)
	print 'Movement: Moved Once'
	pass


# print current coords once received data from topic
def currentCoordsCallback(times):

	Times = times.data
	if Times == 1:
		readCurrentCoords()
	elif Times < 0:
		pass
	else:
		for i in range(0,Times):
			readCurrentCoords()
			time.sleep(1)


# print current angles once received data from topic
def readAnglesCallback(times):

	Times = times.data
	if Times == 1:
		readCurrentAngles()
	elif Times < 0:
		pass
	else:
		for i in range(0,Times):
			readCurrentAngles()
			time.sleep(1)


# print stoppers status once received data from topic
def stopperStatusCallback(times):
	Times = times.data
	if Times == 1:
		readStopperStatus()
	elif Times < 0:
		pass
	else:
		for i in range(0,Times):
			readStopperStatus()
			time.sleep(1)


# monitor mode for listening to all topics
def listener():
	print ' '
	print 'Begin monitor mode - listening to all fucntional topics'
	print '======================================================='
	print '         Use rqt_graph to check the connection         '
	print '======================================================='

	rospy.init_node('uarm_core',anonymous=True)

	rospy.Subscriber("uarm_status",String, attchCallback)
	rospy.Subscriber("pump_control",UInt8, pumpCallack)
	rospy.Subscriber("pump_str_control",String, pumpStrCallack)

	rospy.Subscriber("read_coords",Int32, currentCoordsCallback)
	rospy.Subscriber("read_angles",Int32, readAnglesCallback)
	rospy.Subscriber("stopper_status",Int32, stopperStatusCallback)

	rospy.Subscriber("write_angles",Angles, writeAnglesCallback)
	rospy.Subscriber("move_to",Coords, moveToCallback)
	rospy.Subscriber("move_to_time",CoordsWithTime, moveToTimeCallback)
	rospy.Subscriber("move_to_time_s4",CoordsWithTS4, moveToTimeAndS4Callback)

	rospy.spin()
	pass


# show eroors
def processFailedNum(failed_number):

	if failed_number > 20 and failed_number < 26:
		print 'ERROR: Input Connection Address Is Incorrect'
	if failed_number == 20:
		print 'uArm: Please Connect uArm first '


if __name__ == '__main__':

	try:
		# Connect uarm first
		return_value = connectFcn()

		# Control uarm through commands
		controlFcn()

		# Monitor mode
		if listenerFcn == True:
			listener()

	except:
		processFailedNum(failed_number)
		print 'ERROR: Execution Failed'
		pass

	finally:
		print 'DONE: Program Stopped'
		pass
