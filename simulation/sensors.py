import numpy as np
from math import fmod, floor, sin, cos, pi
from constants import *

class WifiSensor(object):

	def __init__(self, sensor_grid, resolution):

		#Initial values of the sensor reading at a given point.
		#Should be numpy array.
		self.sensor_grid = sensor_grid
		#Ratio of sensor grid distances to millimeters.
		#E.g. if sensor was recorded at every meter then resolution would be 1000.
		self.resolution = resolution
		self.sim_time = 0

	def getReading(self, x, y):
		#Returns the sensor reading at a specific location.
		#Possibly interpolating if point lies between measurements.

		#Scale x and y to sensor_grid distances
		x = float(x) / self.resolution
		y = float(y) / self.resolution

		assert (x >= 0 and x <= self.sensor_grid.shape[0] - 1), "x given to wifi sensor is out of bounds."
		assert (y >= 0 and y <= self.sensor_grid.shape[1] - 1), "y given to wifi sensor is out of bounds."

		#Find the sensor grid indicies around the robot position
		xi_l = int(floor(x))
		xi_h = xi_l + 1
		yi_l = int(floor(y))
		yi_h = yi_l + 1

		#Find how close the actual x and y are.
		xi_l_diff = x - xi_l
		xi_h_diff = 1 - xi_l_diff
		yi_l_diff = y - yi_l
		yi_h_diff = 1 - yi_l_diff

		#Detect if on edge and perform linear interpolation
		x_edge = xi_l == (self.sensor_grid.shape[0] - 1)
		y_edge = yi_l == (self.sensor_grid.shape[1] - 1)

		if x_edge and y_edge:
			return self.sensor_grid[xi_l, yi_l]
		elif x_edge:
			y_l = self.sensor_grid[xi_l, yi_l]
			y_h = self.sensor_grid[xi_l, yi_h]
			return y_l * yi_h_diff + y_h * yi_l_diff
		elif y_edge:
			x_l = self.sensor_grid[xi_l, yi_l]
			x_h = self.sensor_grid[xi_h, yi_l]
			return x_l * xi_h_diff + x_h * xi_l_diff

		#Get the values at the above indicies
		x_l_y_l = self.sensor_grid[xi_l, yi_l]
		x_h_y_l = self.sensor_grid[xi_h, yi_l]
		x_l_y_h = self.sensor_grid[xi_l, yi_h]
		x_h_y_h = self.sensor_grid[xi_h, yi_h]

		#Return bilinear interpolation
		return (x_l_y_l * xi_h_diff * yi_h_diff + 
				x_h_y_l * xi_l_diff * yi_h_diff +
				x_l_y_h * xi_h_diff * yi_l_diff +
				x_h_y_h * xi_l_diff * yi_l_diff)

	def getNoisyReading(x,y):
		return self.getReading(x, y)

	def evolveGrid(self):
		#If the sensor grid changes over time, capture it here.
		self.sim_time += 1

class Wall(object):

	def __init__(self, xl, xh, yl, yh, resolution, offx, offy):
		#xl, xh, yl, and yh define the bounding box of the simulation
		#resolution is the ration of the measured distances to millimeters.
		self.xl = xl * resolution
		self.xh = xh * resolution
		self.yl = yl * resolution
		self.yh = yh * resolution
		#offset of ultrasound on robot
		self.offx = offx * resolution
		self.offy = offy * resolution

	def isInWall(self, x, y):
		xl_wall = (x - float(ROBOTDIAM) / 2) <= (self.xl)
		xh_wall = (x + float(ROBOTDIAM) / 2) >= (self.xh)
		yl_wall = (y - float(ROBOTDIAM) / 2) <= (self.yl)
		yh_wall = (y + float(ROBOTDIAM) / 2) >= (self.yh)

		return (xl_wall or xh_wall or yl_wall or yh_wall)

	def distToWall(self, x, y, orientation):
		x += self.offx
		y += self.offy
		d = []
		if sin(orientation) != 0:
			d.append((x - self.xl) / sin(orientation))
			d.append((self.xh - x) / sin(orientation + pi))
		if cos(orientation) != 0:
			d.append((y - self.yl) / cos(oreintation + pi))
			d.append((self.yh - y) / cos(orientation))

		nearest = None
		for dist in d:
			if dist >=0 and ((nearest is None) or (nearest >= dist)):
				nearest = dist

		return nearest

		def noisyDistToWall(x, y, orientation):
			return self.distToWall(x, y, orientation)

#Examples
wifi_array = np.array([[1,2,3,4],[2,3,4,5],[3,4,5,6], [4,5,6,7]])

wifi = WifiSensor(wifi_array, 1000)

wall = Wall(0, 3, 0, 3, 1000, 0, 0)