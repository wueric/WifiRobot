import numpy as np
from sensors import wifi, wall
from math import cos, sin, pi, atan2
from constants import *

class Robot(object):

	def __init__(self, name, x, y, orientation, sense_array):
		self.name = name
		self.x = x
		self.y = y
		self.orientation = orientation
		self.sense_array = sense_array
		#Specify initial values of state variables you need for control
		#Currently (maneuver start time, maneuver start angle, maneuver substep, maneuver iteration)
		self.control_state = [0,0,0,3]

	def Magnetometer(self):
		y_off = sin(self.orientation) + OROFFY
		x_off = cos(self.orientation) + OROFFX
		return (atan2(y_off, x_off)) % (2*pi)

	def angleDif(self, a, b):
		if a < b :
			return min(a-b, (b+2*pi) - a)
		else:
			return min(b-a, (a+2*pi) - b)

	def control(self, time):
		#Wheel speed in mm/sec
		state = self.control_state[2]
		man_time = time - self.control_state[0]
		#Start
		if state == 0:
			self.control_state[0] = time + 1
			self.control_state[1] = self.Magnetometer()
			self.control_state[2] = state + 1
			return 0, 0

		#Stop
		elif state == 9:
			return 0, 0

		else:
			#Long Forward
			if (state == 1) or (state == 5):
				if man_time <= 10000:
					return 100, 100
				else:
					self.control_state[0] = time + 1
					self.control_state[1] = self.Magnetometer()
					self.control_state[2] = state + 1
					return 0, 0

			#Right Turn
			elif (state == 2) or (state == 4):
				goal_angle = (self.control_state[1] + (pi/2)) % (2*pi)
				if abs(self.angleDif(self.Magnetometer(), goal_angle)) > .03 :
					return 100, -100
				else:
					self.control_state[0] = time + 1
					self.control_state[1] = self.Magnetometer()
					self.control_state[2] = state + 1
					return 0, 0

			#Short Forward
			elif (state == 3) or (state == 7):
				if man_time <= 2000:
					return 100, 100
				else:
					self.control_state[0] = time + 1
					self.control_state[1] = self.Magnetometer()
					self.control_state[2] = state + 1
					return 0, 0

			#Left Turn
			elif (state == 6) or (state == 8):
				goal_angle = (self.control_state[1] - (pi/2)) % (2*pi)
				if abs(self.angleDif(self.Magnetometer(), goal_angle)) > .03 :
					return -100, 100
				else:
					self.control_state[0] = time + 1
					self.control_state[1] = self.Magnetometer()
					self.control_state[2] = state + 1
					if state == 8:
						self.control_state[3] = self.control_state[3] - 1
						if self.control_state[3] == 0:
							self.control_state[2] = 9
						else:
							self.control_state[2] = 1
					return 0, 0

		return None, None


	def motionStep(self, time):
		wall = self.sense_array[0]
		l_speed, r_speed = self.control(time)
		'''
		if not (l_speed == 100 and r_speed == 100):
			print l_speed, r_speed
		'''
		dx, dy, dor = self.kinematics(l_speed, r_speed)
		x_new = self.x + dx
		y_new = self.y + dy
		if not wall.isInWall(x_new, y_new):
			self.x = x_new
			self.y = y_new
			self.orientation = (self.orientation + dor) % (2*pi)

		
	def kinematics(self, l_speed, r_speed):
		#Convert mm/sec to mm/millisecond.
		l_speed = float(l_speed) / 1000
		r_speed = float(r_speed) / 1000
		#How the robot would move if unobstructed.
		dx = (float(l_speed + r_speed) / 2) * cos(self.orientation)
		dy = (float(l_speed + r_speed) / 2) * sin(self.orientation)
		dor = float(r_speed - l_speed) / WHEELSPACE
		return dx, dy, dor