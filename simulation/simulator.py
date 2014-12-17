import numpy as np
import sensors
import robot
from math import pi
import csv
import constants

wifi_array = np.random.rand(4,4)

wifi = sensors.WifiSensor(wifi_array, 1000)

wall = sensors.Wall(0, 3, 0, 3, 1000, 0, 0)

irobot = robot.Robot("test", 500, 500, 0, [wall, wifi_array])



with open('NoOff03Tol.csv', 'wb') as csvfile:
	data = csv.writer(csvfile, delimiter=',')
#elapse time (millisecond at a time)
	for time in xrange(150000):
		#Print location once per second.
		if (time % 100) == 0:
			data.writerow([time, irobot.x, irobot.y, irobot.orientation])
		if (time % 1000) == 0:
			x = irobot.x
			y = irobot.y
			print("Time: " + str(time/1000) + "  Location: (" + str(round(x)) + "," + str(round(y)) + ")  Orientation: " + str(round(irobot.orientation, 3)) + "  WiFi: " + str(round(wifi.getReading(x,y), 3)))
		irobot.motionStep(time)
