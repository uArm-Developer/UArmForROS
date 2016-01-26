
from pyfirmata import Arduino, util
import math
import time


# PUMP INFORMATION
PUMP_EN = 6
VALVE_EN =5

# UARM SPECIFICATIONS
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


class Uarm(object):
	
    
	uarm = None
	kAddrOffset = 90
	kAddrAandB = 60

	uarm_status = 0
	pin2_status = 0
	coord = {}
	g_interpol_val_arr = {}
	angle = {}

	attachStaus = 0

	def __init__(self,port):
		
		self.uarm = Arduino(port)
		self.uarmDetach()
		

	def servoAttach(self,servo_number):
		if servo_number == 1:
			self.servo_base = self.uarm.get_pin('d:11:s')
		elif servo_number == 2:
			self.servo_left = self.uarm.get_pin('d:13:s')
		elif servo_number == 3:
			self.servo_right = self.uarm.get_pin('d:12:s')
		elif servo_number == 4:
			self.servo_end = self.uarm.get_pin('d:10:s')
		else:
			return	


	def servoDetach(self,servo_number):
		if servo_number == 1:
			self.uarm.servoDetach(11)
		elif servo_number == 2:
			self.uarm.servoDetach(13)
		elif servo_number == 3:
			self.uarm.servoDetach(12)
		elif servo_number == 4:
			self.uarm.servoDetach(10)
		else:
			return	
	

	def uarmDisconnect(self):
		self.uarm.exit()
	

	def uarmAttach(self):

		curAngles = {}

		if self.uarm_status == 0:
			for n in range(1,5):
				curAngles[n] = self.readAngle(n)
			time.sleep(0.1)
			n = 1
			while n<5:
				self.servoAttach(n)
				n += 1
			time.sleep(0.1)
			self.writeAngle(curAngles[1],curAngles[2],curAngles[3],curAngles[4])
			self.uarm_status = 1
	

	def uarmDetach(self):
		n = 1
		while n<5:
			self.servoDetach(n)
			n += 1
		self.uarm_status = 0


	def angleConstrain(self,Angle):
		if Angle <0:
			return 0
		elif Angle >180:
			return 180
		else:	
			return Angle


	def writeServoAngleRaw(self,servo_number,Angle):
		
		if servo_number == 1:
			self.servo_base.write(self.angleConstrain(round(Angle)))
		elif servo_number == 2:
			self.servo_left.write(self.angleConstrain(round(Angle)))
		elif servo_number == 3:
			
			self.servo_right.write(self.angleConstrain(round(Angle)))
		elif servo_number == 4:
			
			self.servo_end.write(self.angleConstrain(round(Angle)))
		else:
			return


	def writeServoAngle(self,servo_number,Angle):
		
		if servo_number == 1:
			self.servo_base.write(self.angleConstrain(Angle+ self.readServoOffset(servo_number))) 
		elif servo_number == 2:	
			self.servo_left.write(self.angleConstrain(Angle+ self.readServoOffset(servo_number)))
		elif servo_number == 3:
			self.servo_right.write(self.angleConstrain(Angle+ self.readServoOffset(servo_number))) 
		elif servo_number == 4:
			self.servo_end.write(self.angleConstrain(Angle+ self.readServoOffset(servo_number)))
		else:
			return


	def writeAngle(self,servo_1,servo_2,servo_3,servo_4):

		servoAngles = {}
		servoAngles[1] = servo_1 + self.readServoOffset(1)
		servoAngles[2] = servo_2 + self.readServoOffset(2)
		servoAngles[3] = servo_3 + self.readServoOffset(3)
		servoAngles[4] = servo_4 + self.readServoOffset(4)

		self.servo_base.write(self.angleConstrain(servoAngles[1])) 
		self.servo_left.write(self.angleConstrain(servoAngles[2]))
		self.servo_right.write(self.angleConstrain(servoAngles[3]))
		self.servo_end.write(self.angleConstrain(servoAngles[4])) 


	def writeAngleRaw(self,servo_1,servo_2,servo_3,servo_4):
		
		self.servo_base.write(self.angleConstrain(servo_1))
		self.servo_left.write(self.angleConstrain(servo_2))
		self.servo_right.write(self.angleConstrain(servo_3))
		self.servo_end.write(self.angleConstrain(servo_4))
		

	def readAnalog(self,servo_number):
	
		if servo_number == 1:
			for i in range(1,4):
				data = self.uarm.readAnalogPin(2)
			return data
		elif servo_number == 2:
			for i in range(1,4):
				data = self.uarm.readAnalogPin(0)
			return data
		elif servo_number == 3:
			for i in range(1,4):
				data = self.uarm.readAnalogPin(1)
			return data
		elif servo_number == 4:
			for i in range(1,4):
				data = self.uarm.readAnalogPin(3)
			return data
		else:
			return
	

	def readServoOffset(self,servo_number):
		if servo_number ==1 or servo_number ==2 or servo_number ==3:
			addr = self.kAddrOffset + (servo_number - 1)*2
			servo_offset = (self.uarm.readEEPROM(addr+1))/10.00
			if(self.uarm.readEEPROM(addr) == 0):
				servo_offset *= -1
			return servo_offset
		elif servo_number == 4:
			return 0
		else:
			pass
	

	def readToAngle(self,input_analog,servo_number,tirgger):
		addr = self.kAddrAandB + (servo_number-1)*6

		data_a = (self.uarm.readEEPROM(addr+1)+ (self.uarm.readEEPROM(addr+2)*256))/10.0

		if (self.uarm.readEEPROM(addr) == 0):
			data_a = -data_a
		data_b = (self.uarm.readEEPROM(addr+4)+ (self.uarm.readEEPROM(addr+5)*256))/100.0
		
		if (self.uarm.readEEPROM(addr+3) == 0):
			data_b = -data_b
		
		if tirgger == 0 :
			return (data_a + data_b *input_analog) - self.readServoOffset(servo_number)
		elif tirgger == 1:
			return (data_a + data_b *input_analog)
		else:
			pass


	def fwdKine(self,theta_1,theta_2,theta_3):
		g_l3_1 = MATH_L3 * math.cos(theta_2/MATH_TRANS)
		g_l4_1 = MATH_L4 * math.cos(theta_3 / MATH_TRANS);
  		g_l5 = (MATH_L2 + MATH_L3*math.cos(theta_2 / MATH_TRANS) + MATH_L4*math.cos(theta_3 / MATH_TRANS));

		self.coord[1] = -math.cos(abs(theta_1 / MATH_TRANS))*g_l5;
		self.coord[2] = -math.sin(abs(theta_1 / MATH_TRANS))*g_l5;
		self.coord[3] = MATH_L1 + MATH_L3*math.sin(abs(theta_2 / MATH_TRANS)) - MATH_L4*math.sin(abs(theta_3 / MATH_TRANS));
		return self.coord


	def currentCoord(self):
		return self.fwdKine(self.readAngle(1),self.readAngle(2),self.readAngle(3))


	def currentX(self):
		self.currentCoord()
		return self.coord[1]


	def currentY(self):
		self.currentCoord()
		return self.coord[2]


	def currentZ(self):
		self.currentCoord()
		return self.coord[3]


	def readAngle(self,servo_number):
		if servo_number == 1:
			return self.readToAngle(self.readAnalog(1),1,0)
		elif servo_number == 2:
			return self.readToAngle(self.readAnalog(2),2,0)
		elif servo_number == 3:
			return self.readToAngle(self.readAnalog(3),3,0)
		elif servo_number == 4:
			return self.readToAngle(self.readAnalog(4),4,0)
		else:
			pass  


	def readAngleRaw(self,servo_number):
		if servo_number == 1:
			return self.readToAngle(self.readAnalog(1),1,1)
		elif servo_number == 2:
			return self.readToAngle(self.readAnalog(2),2,1)
		elif servo_number == 3:
			return self.readToAngle(self.readAnalog(3),3,1)
		elif servo_number == 4:
			return self.readToAngle(self.readAnalog(4),4,1)
		else:
			pass  	
		

	def interpolation(self,init_val,final_val):
		#by using the formula theta_t = l_a_0 + l_a_1 * t + l_a_2 * t^2 + l_a_3 * t^3
  		#theta(0) = init_val; theta(t_f) = final_val
		#theta_dot(0) = 0; theta_dot(t_f) = 0

		l_time_total = 1

		l_a_0 = init_val;
		l_a_1 = 0;
		l_a_2 = (3 * (final_val - init_val)) / (l_time_total*l_time_total);
		l_a_3 = (-2 * (final_val - init_val)) / (l_time_total*l_time_total*l_time_total);
		
		i = 0
		while i<51:
			
			l_t_step = (l_time_total / 50.0) *i
			self.g_interpol_val_arr[i] = l_a_0 + l_a_1 * (l_t_step) + l_a_2 * (l_t_step *l_t_step ) + l_a_3 * (l_t_step *l_t_step *l_t_step);
			i+=1	
		return self.g_interpol_val_arr


	def pumpOn(self):
		self.uarm.digital[PUMP_EN].write(1)
		self.uarm.digital[VALVE_EN].write(0)


	def pumpOff(self):
		self.uarm.digital[PUMP_EN].write(0)
		self.uarm.digital[VALVE_EN].write(1)
		time.sleep(0.02)
		self.uarm.digital[VALVE_EN].write(0)
		

	def ivsKine(self, x, y, z):
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

		if g_theta_3 < 0 :
			pass
		else:
			self.fwdKine(g_theta_1,g_theta_2, g_theta_3)
			if (self.coord[2]>y+0.1) or (self.coord[2]<y-0.1):
				g_theta_2 = 180 - g_theta_2

		if(math.isnan(g_theta_1) or math.isinf(g_theta_1)):
			g_theta_1 = self.readAngle(1)
		if(math.isnan(g_theta_2) or math.isinf(g_theta_2)):
			g_theta_2 = self.readAngle(2)
		if(math.isnan(g_theta_3) or math.isinf(g_theta_3) or (g_theta_3<0)):
			g_theta_3 = self.readAngle(3)
		
		self.angle[1] = g_theta_1
		self.angle[2] = g_theta_2
		self.angle[3] = g_theta_3
		return self.angle

		pass


	def moveToWithS4(self,x,y,z,timeSpend,servo_4_relative,servo_4):
		
		if self.uarm_status == 0:
			self.uarmAttach()
			self.uarm_status = 1

		curXYZ = self.currentCoord()

		x_arr = {}
		y_arr = {}
		z_arr = {}

		if time >0:
			self.interpolation(curXYZ[1],x)
			for n in range(0,50):
				x_arr[n] = self.g_interpol_val_arr[n]

			self.interpolation(curXYZ[2],y)
			for n in range(0,50):
				y_arr[n] = self.g_interpol_val_arr[n]

			self.interpolation(curXYZ[3],z)
			for n in range(0,50):
				z_arr[n] = self.g_interpol_val_arr[n]
		
			for n in range(0,50):
				self.ivsKine(x_arr[n],y_arr[n],z_arr[n])
				self.writeAngle(self.angle[1],self.angle[2],self.angle[3],self.angle[1]*servo_4_relative+servo_4)
				time.sleep(timeSpend/50.0)

		elif time == 0:
			self.ivsKine(x,y,z)
			self.writeAngle(self.angle[1],self.angle[2],self.angle[3],self.angle[1]*servo_4_relative+servo_4)

		else:
			pass


	def moveTo(self,x,y,z):
		
		if self.uarm_status == 0:
			self.uarmAttach()
			self.uarm_status = 1


		curXYZ = self.currentCoord()

		x_arr = {}
		y_arr = {}
		z_arr = {}

		self.interpolation(curXYZ[1],x)
		for n in range(0,50):
			x_arr[n] = self.g_interpol_val_arr[n]

		self.interpolation(curXYZ[2],y)
		for n in range(0,50):
			y_arr[n] = self.g_interpol_val_arr[n]

		self.interpolation(curXYZ[3],z)
		for n in range(0,50):
			z_arr[n] = self.g_interpol_val_arr[n]
	
		for n in range(0,50):
			self.ivsKine(x_arr[n],y_arr[n],z_arr[n])
			self.writeAngle(self.angle[1],self.angle[2],self.angle[3],0)
		
			time.sleep(0.04)


	def moveToWithTime(self,x,y,z,timeSpend):
		
		if self.uarm_status == 0:
			self.uarmAttach()
			self.uarm_status = 1

		curXYZ = self.currentCoord()
		x_arr = {}
		y_arr = {}
		z_arr = {}

		if time >0:
			self.interpolation(curXYZ[1],x)
			for n in range(0,50):
				x_arr[n] = self.g_interpol_val_arr[n]

			self.interpolation(curXYZ[2],y)
			for n in range(0,50):
				y_arr[n] = self.g_interpol_val_arr[n]

			self.interpolation(curXYZ[3],z)
			for n in range(0,50):
				z_arr[n] = self.g_interpol_val_arr[n]
		
			for n in range(0,50):
				self.ivsKine(x_arr[n],y_arr[n],z_arr[n])
				self.writeAngle(self.angle[1],self.angle[2],self.angle[3],0)
				time.sleep(timeSpend/50.0)

		elif time == 0:
			
			self.ivsKine(x,y,z)
			self.writeAngle(self.angle[1],self.angle[2],self.angle[3],0)

		else:
			pass

	def moveToAtOnce(self,x,y,z):
		
		if self.uarm_status == 0:
			self.uarmAttach()
			self.uarm_status = 1

		self.ivsKine(x,y,z)
		self.writeAngle(self.angle[1],self.angle[2],self.angle[3],0)

	
	def moveRelative(self,x,y,z,time,servo_4_relative,servo_4):
		pass
		

	def stopperStatus(self):
		val = self.uarm.pumpStatus(0)
		
		if val ==2 or val ==1:
			return val-1
		else:
			print 'ERROR: Stopper is not deteceted'
			return -1
		












