"""
@author: Deepak

"""
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

x = 0
y = 0
z = 0
z_orient = 0

pitch = 0
roll = 0
yaw_velocity = 0
z_velocity = 0

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

	#rospy.loginfo('before sleep')
	time.sleep(1)
	output()
	#rospy.loginfo('before sendtakeoff')
	controller.SendTakeoff()
	#rospy.loginfo('before setcommand')
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

	#rospy.loginfo('x: {}, y: {}, z: {}'.format(x,y,z))

	while z_up == True or z_down == True:
		orient()
		if math.fabs(z - h) < 1:
			z_speed = 1
		
		#rospy.loginfo('x: {}, y: {}, z: {}'.format(x,y,z))
		
		# z pos
		if z_up == True:
			z_velocity += z_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			z_velocity -= z_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if z > h:
				z_up = False
		# z neg
		elif z_down == True:
			z_velocity -= z_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			z_velocity += z_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if z < h:
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

	#rospy.loginfo('x: {}, y: {}, z: {}'.format(x,y,z))

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

		#rospy.loginfo('x: {}, y: {}, z: {}'.format(x,y,z))
		# x pos and y pos
		if x_up == True and y_up == True:
			#rospy.loginfo('x up y up')
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
		# x pos and y neg
		elif x_up == True and y_down == True:
			#rospy.loginfo('x up y down')
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
		# x neg and y pos
		elif x_down == True and y_up == True:
			#rospy.loginfo('x down y up')
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
		# x neg and y neg
		elif x_down == True and y_down == True:
			#rospy.loginfo('x down y down')
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
		# x pos
		elif x_up == True:
			#rospy.loginfo('x up')
			pitch += x_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			pitch -= x_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if x > check_x:
				x_up = False
		# x neg
		elif x_down == True:
			#rospy.loginfo('x down')
			pitch -= x_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			pitch += x_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if x < check_x:
				x_down = False
		# y pos
		elif y_up == True:
			#rospy.loginfo('y up')
			roll += y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			roll -= y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if y > check_y:
				y_up = False
		# y neg
		elif y_down == True:
			#rospy.loginfo('y down')
			roll -= y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			time.sleep(1)
			roll += y_speed
			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)
			if y < check_y:
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

	

if __name__ == '__main__':
	import sys
	rospy.init_node('drone_movement')
	with open('input.txt') as f:
		checkpoints = [tuple(map(float, i.split(','))) for i in f]
	fo = open("output.txt", "wb")

        controller = BasicDroneController()
	rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
	rospy.Subscriber("/clock", Clock, timeback)

	#movement commands
	launch()
	setHeight(9)
	moveToXY(checkpoints[0][0], checkpoints[0][1])
	setHeight(checkpoints[0][2])
	output()
	setHeight(9)
	moveToXY(checkpoints[1][0], checkpoints[1][1])
	setHeight(checkpoints[1][2])
	output()
	land()
	output()
	
	fo.close()
	#rospy.spin()

