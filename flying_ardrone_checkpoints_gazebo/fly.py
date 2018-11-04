#!/usr/bin/env python

import math
import time
import rospy
import roslib; roslib.load_manifest('keyboard_test')

#from drone_video_display import DroneVideoDisplay
from gazebo_msgs.msg import ModelStates
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
	output()
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
			if z > (h - 0.5):
				z_up = False

		elif z_down == True:
			z_velocity -= z_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			z_velocity += z_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if z < (h + 0.5):
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
			x_speed = 0.2
		elif math.fabs(x - check_x) < 3:
			x_speed = 1
		if math.fabs(y - check_y) < 1:
			y_speed = 0.2
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
			if x > (check_x - .5):
				x_up = False
			if y > (check_y - .5):
				y_up = False

		elif x_up == True and y_down == True:
			pitch += x_speed
			roll -= y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			pitch -= x_speed
			roll += y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if x > (check_x - .5):
				x_up = False
			if y < (check_y + .5):
				y_down = False

		elif x_down == True and y_up == True:
			pitch -= x_speed
			roll += y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			pitch += x_speed
			roll -= y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if x < (check_x + .5):
				x_down = False
			if y > (check_y - .5):
				y_up = False

		elif x_down == True and y_down == True:
			pitch -= x_speed
			roll -= y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			pitch += x_speed
			roll += y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if x < (check_x + .5):
				x_down = False
			if y < (check_y + .5):
				y_down = False

		elif x_up == True:
			pitch += x_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			pitch -= x_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if x > (check_x - .5):
				x_up = False

		elif x_down == True:
			pitch -= x_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			pitch += x_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if x < (check_x + .5):
				x_down = False
		elif y_up == True:
			roll += y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			roll -= y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if y > (check_y - .5):
				y_up = False

		elif y_down == True:
			roll -= y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			roll += y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if y < (check_y + .5):
				y_down = False
		else:
			break

def output():
	global x
	global y
	global z
	global sec

	m, s = divmod(sec, 60)
	h, m = divmod(m, 60)
	
	fo.write("(%.5f, %.5f, %.5f) (%02i:%02i:%02i)\n" % (x,y,z,h,m,s))

def distance(dis_x, dis_y, dis_z):
	global dis_up
	dis_up = 7.1
	global x
	global y
	global z
	
	dist = math.sqrt(math.fabs(dis_x-x)*math.fabs(dis_x-x) + math.fabs(dis_y-y)*math.fabs(dis_y-y))+dis_up+ math.fabs(z-dis_z)
	return dist	

if __name__ == '__main__':
	import sys
	rospy.init_node('drone_movement')
	with open('input.txt') as f:
		checkpoints = [tuple(map(float, i.split(','))) for i in f]
	fo = open("output.txt", "wb")
	
        controller = BasicDroneController()
	rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
	rospy.Subscriber("/clock", Clock, timeback)
	

	launch()
	dist1=[]
	num_cp=10				# total number of available checkpoints

	while num_cp != 0:
		for k in range(0,num_cp):
			dist1.append(distance(checkpoints[k][0],checkpoints[k][1],checkpoints[k][2]))
			
		dist_nearest_cp, i = min((val, idx) for (idx, val) in enumerate(dist1))
	
		print "\n \n Next closest distance is ", dist_nearest_cp
		print "Checking travelling time to ", checkpoints[i]

		dist_nearestcp_sp = math.sqrt(math.fabs(checkpoints[i][0])*math.fabs(checkpoints[i][0]) + math.fabs(checkpoints[i][1])*math.fabs(checkpoints[i][1]))+7.1+7.1-checkpoints[i][2]
		check_time = (dist_nearest_cp + dist_nearestcp_sp)/speed
		
		print "Time remaining = ", (time_limit - sec)
		print "Estimated time to do another check point = ", check_time

		if check_time > (time_limit - sec):
			break
		print "==>> Travelling to ", checkpoints[i]
		setHeight(7.1)
		moveToXY(checkpoints[i][0], checkpoints[i][1])
		setHeight(checkpoints[i][2])
		moveToXY(checkpoints[i][0], checkpoints[i][1])
		output()

		checkpoints.remove(checkpoints[i])
		del dist1[:]
		num_cp-=1

	setHeight(7.1)
	print "\n \n Going back home"
	moveToXY(0, 0)
	setHeight(.5)
	moveToXY(0, 0)

	land()
	output()
	
	fo.close()

