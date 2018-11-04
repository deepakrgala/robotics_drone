#!/usr/bin/env python

import math
import time
import rospy
import roslib; roslib.load_manifest('keyboard_test')


#from drone_video_display import DroneVideoDisplay
from gazebo_msgs.msg import ModelStates
#from ardrone_autonomy.msg import Navdata 
from ardrone_autonomy.msg import navdata_gps

from rosgraph_msgs.msg import Clock
from drone_controller import BasicDroneController
#from PySide import QtCore, QtGui

sec = 0
dist=0
dist_up=0
x = 0
y = 0
z = 0
z_orient = 0

pitch = 0
roll = 0
yaw_velocity = 0
z_velocity = 0
t_start = 0
speed = 0.456 					#estimated average speed in m/s
time_limit = 0


def ReceiveNavdata(self,navdata):
	self.status = navdata.state


def timeback(msg):
	global sec
	sec = msg.clock.secs	

def callback(msg):
	global x
	global y
	global z
	global z_orient

	x=msg.pose[11].position.x
        y=msg.pose[11].position.y
        z=msg.pose[11].position.z
        z_orient=msg.pose[11].orientation.z

def launch():
	#rospy.loginfo('x: {}, y: {}, z: {}'.format(x,y,z))
	global pitch
	global roll
	global yaw_velocity
	global z_velocity
	global time_limit

	time.sleep(1)
	time_limit = sec + 900					# set for time limits
#	output()
	controller.SendTakeoff()
	controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)

def land():
	global pitch
	global roll
	global yaw_velocity
	global z_velocity
	global z

	controller.SendLand()
	controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
	while z > 0.1:
		i=0

def orient():
	global pitch
	global roll
	global yaw_velocity
	global z_velocity
	global z_orient
	
	if z_orient > 0:
		yaw_velocity -= 1
		controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
		time.sleep(0.1)
		yaw_velocity += 1
		controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
	elif z_orient < 0:
		yaw_velocity += 1
		controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
		time.sleep(0.1)
		yaw_velocity -= 1
		controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
	

def setHeight(h):
	global pitch
	global roll
	global yaw_velocity
	global z_velocity
	global z

	z_speed = 3
	z_up = z < h
	z_down = z > h

	while z_up == True or z_down == True:
		orient()
		if math.fabs(z - h) < 1:
			z_speed = 1
		
		if z_up == True:
			z_velocity += z_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			z_velocity -= z_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if z > h:
				z_up = False

		elif z_down == True:
			z_velocity -= z_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			z_velocity += z_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if z < h+0.5:
				z_down = False
		else:
			break

def moveToXY(check_x, check_y):
	global pitch
	global roll
	global yaw_velocity
	global z_velocity

	global x
	global y

	x_up = x < check_x
	x_down = x > check_x
	y_up = y < check_y
	y_down = y > check_y

	x_speed = 3
	y_speed = 3

	while (x_up == True or x_down == True) or (y_up == True or y_down == True):
		orient()
		if math.fabs(x - check_x) < 1:
			x_speed = 0.5
		elif math.fabs(x - check_x) < 3:
			x_speed = 1
		if math.fabs(y - check_y) < 1:
			y_speed = 0.5
		elif math.fabs(y - check_y) < 3:
			y_speed = 1

		if x_up == True and y_up == True:
			pitch += x_speed
			roll += y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			pitch -= x_speed
			roll -= y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if x > check_x:
				x_up = False
			if y > check_y:
				y_up = False

		elif x_up == True and y_down == True:
			pitch += x_speed
			roll -= y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			pitch -= x_speed
			roll += y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if x > check_x:
				x_up = False
			if y < check_y:
				y_down = False

		elif x_down == True and y_up == True:
			pitch -= x_speed
			roll += y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			pitch += x_speed
			roll -= y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if x < check_x:
				x_down = False
			if y > check_y:
				y_up = False

		elif x_down == True and y_down == True:
			pitch -= x_speed
			roll -= y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			pitch += x_speed
			roll += y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if x < check_x:
				x_down = False
			if y < check_y:
				y_down = False

		elif x_up == True:
			pitch += x_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			pitch -= x_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if x > check_x:
				x_up = False

		elif x_down == True:
			pitch -= x_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			pitch += x_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if x < check_x:
				x_down = False
		elif y_up == True:
			roll += y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			roll -= y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if y > check_y:
				y_up = False

		elif y_down == True:
			roll -= y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			roll += y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if y < check_y:
				y_down = False
		else:
			break
def gps_data(position):
	global x
	global y
	
	x=position.latitude
	y=position.longitude
	print "x= ", x 
	print "y= ", y
	print "\n \n"	
	print "check"
	

if __name__ == '__main__':
	import sys
	rospy.init_node('drone_movement')
        controller = BasicDroneController()
	#rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
	rospy.Subscriber("/clock", Clock, timeback)

	#Subscribe to the /ardrone/navdata topic, of message type Navdata, and call self.ReceiveNavdata when a message is received
#	rospy.Subscriber('/ardrone/navdata',Navdata,drone_data)
	rospy.Subscriber('/ardrone/navdata_gps',navdata_gps, gps_data)	

	launch()
	time.sleep(2)
	
	land()

	

