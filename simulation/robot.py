import numpy as np
from sensors import wifi, wall
from math import cos, sin, pi
from constants import *

class Robot(object):

	def __init__(self, name, x, y, orientation, sense_array):
		self.name = name
		self.x = x
		self.y = y
		self.orientation = orientation
		self.sense_array = sense_array

	def control(self, time):
		#Wheel speed in mm/sec
		return 224, 200

	def motionStep(self, time):
		wall = self.sense_array[0]
		l_speed, r_speed = self.control(time)
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