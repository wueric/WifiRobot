import numpy as np
import sensors
import robot
from math import pi

wifi_array = np.random.rand(11,11)

wifi = sensors.WifiSensor(wifi_array, 1000)

wall = sensors.Wall(0, 10, 0, 10, 1000, 0, 0)

irobot = robot.Robot("test", 6000, 6000, (3*pi)/4, [wall, wifi_array])

#elapse time (millisecond at a time)
for time in xrange(30000):
	#Print location once per second.
	if (time % 1000) == 0:
		x = irobot.x
		y = irobot.y
		print("Time: " + str(time/1000) + "  Location: (" + str(round(x)) + "," + str(round(y)) + ")  Orientation: " + str(round(irobot.orientation, 3)) + "  WiFi: " + str(round(wifi.getReading(x,y), 3)))
	irobot.motionStep(time)
