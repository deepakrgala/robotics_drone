#!/usr/bin/env python

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar
# By default it includes no control functionality. The class can be extended to implement key or mouse listeners if required

#For flight/led animations
import os

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('keyboard_test')
import rospy

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image    	 # for receiving the video feed
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# The GUI libraries
from PySide import QtCore, QtGui


# Some Constants
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms
DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected


class DroneVideoDisplay(QtGui.QMainWindow):
	StatusMessages = {
		DroneStatus.Emergency : 'Emergency',
		DroneStatus.Inited    : 'Initialized',
		DroneStatus.Landed    : 'Landed',
		DroneStatus.Flying    : 'Flying',
		DroneStatus.Hovering  : 'Hovering',
		DroneStatus.Test      : 'Test (?)',
		DroneStatus.TakingOff : 'Taking Off',
		DroneStatus.GotoHover : 'Going to Hover Mode',
		DroneStatus.Landing   : 'Landing',
		DroneStatus.Looping   : 'Looping (?)'
		}
	DisconnectedMessage = 'Disconnected'
	UnknownMessage = 'Unknown Status'
	
	def __init__(self):
		# Construct the parent class
		super(DroneVideoDisplay, self).__init__()

		# Setup our very basic GUI - a label which fills the whole window and holds our image
		self.setWindowTitle('AR.Drone Video Feed')
		self.imageBox = QtGui.QLabel(self)
		self.setCentralWidget(self.imageBox)
	
		# Current color to compare with captured color
		self.currentcolor = -1
		self.color1 = 0
		self.color2 = 0
		self.color3 = 0
		self.color4 = 0
		self.trained = False

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
		
		# Holds the image frame received from the drone and later processed by the GUI
		self.image = None
		self.imageLock = Lock()

		self.tags = []
		self.tagLock = Lock()
		
		# Holds the status message to be displayed on the next GUI update
		self.statusMessage = ''

		# Tracks whether we have received data since the last connection check
		# This works because data comes in at 50Hz but we're checking for a connection at 4Hz
		self.communicationSinceTimer = False
		self.connected = False

		# A timer to check whether we're still connected
		self.connectionTimer = QtCore.QTimer(self)
		self.connectionTimer.timeout.connect(self.ConnectionCallback)
		self.connectionTimer.start(CONNECTION_CHECK_PERIOD)
		
		# A timer to redraw the GUI
		self.redrawTimer = QtCore.QTimer(self)
		self.redrawTimer.timeout.connect(self.RedrawCallback)
		self.redrawTimer.start(GUI_UPDATE_PERIOD)

	# Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
	def ConnectionCallback(self):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	def RedrawCallback(self):
		if self.image is not None:
			# We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
			self.imageLock.acquire()
			try:			
					# Convert the ROS image into a QImage which we can display
					image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))
					#qimage = QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888)
					#rgb = qimage.pixel(1,1)
					#self.statusMessage = "Red: {}   Green: {}   Blue: {}".format(QtGui.qRed(rgb),QtGui.qGreen(rgb),QtGui.qBlue(rgb))
					if len(self.tags) > 0:
						self.tagLock.acquire()
						try:
							painter = QtGui.QPainter()
							painter.begin(image)
							painter.setPen(QtGui.QColor(0,255,0))
							painter.setBrush(QtGui.QColor(0,255,0))
							for (x,y,d) in self.tags:
								r = QtCore.QRectF((x*image.width())/1000-DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,DETECT_RADIUS*2,DETECT_RADIUS*2)
								painter.drawEllipse(r)
								painter.drawText((x*image.width())/1000+DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,str(d/100)[0:4]+'m')
							painter.end()
						finally:
							self.tagLock.release()
			finally:
				self.imageLock.release()

			# We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.
			self.resize(image.width(),image.height())
			self.imageBox.setPixmap(image)
			#self.statusMessage = "w: {}  h: {}".format(image.width(),image.height())

		# Update the status bar to show the current drone status & battery level
		self.statusBar().showMessage(self.statusMessage if self.connected else self.DisconnectedMessage)

	def ReceiveImage(self,data):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
		self.imageLock.acquire()
		try:
			self.image = data # Save the ros image for processing by the display thread
			if self.trained:
				#capture current color
				qimage = QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888)
				self.currentcolor = qimage.pixel(320,180)
					#rgb = qimage.pixel(320,180)
				#self.statusMessage = "Red: {}   Green: {}   Blue: {}".format(QtGui.qRed(rgb),QtGui.qGreen(rgb),QtGui.qBlue(rgb))
				currentR = QtGui.qRed(self.currentcolor)
				currentG = QtGui.qGreen(self.currentcolor)
				currentB = QtGui.qBlue(self.currentcolor)
				color1R = QtGui.qRed(self.color1)
				color1G = QtGui.qGreen(self.color1)
				color1B = QtGui.qBlue(self.color1)
				color2R = QtGui.qRed(self.color2)
				color2G = QtGui.qGreen(self.color2)
				color2B = QtGui.qBlue(self.color2)
				color3R = QtGui.qRed(self.color3)
				color3G = QtGui.qGreen(self.color3)
				color3B = QtGui.qBlue(self.color3)
				color4R = QtGui.qRed(self.color4)
				color4G = QtGui.qGreen(self.color4)
				color4B = QtGui.qBlue(self.color4)

				if (currentR > (color1R - 10) and currentR < (color1R + 10)) and (currentG > (color1G - 10) and currentG < (color1G + 10)) and (currentB > (color1B - 10) and currentB < (color1B + 10)):
					self.statusMessage = "color1 seen again"
					os.system("rosservice call /ardrone/setledanimation 0 4 5")
					#os.system("rosservice call /ardrone/setflightanimation 9 0")
					self.color1 = 0
				elif (currentR > (color2R - 10) and currentR < (color2R + 10)) and (currentG > (color2G - 10) and currentG < (color2G + 10)) and (currentB > (color2B - 10) and currentB < (color2B + 10)):
					self.statusMessage = "color2 seen again"
					os.system("rosservice call /ardrone/setledanimation 5 4 5")
					#os.system("rosservice call /ardrone/setflightanimation 13 0")
					self.color2 = 0
				elif (currentR > (color3R - 10) and currentR < (color3R + 10)) and (currentG > (color3G - 10) and currentG < (color3G + 10)) and (currentB > (color3B - 10) and currentB < (color3B + 10)):
					self.statusMessage = "color3 seen again"
					#os.system("rosservice call /ardrone/setledanimation 1 4 5")
					os.system("rosservice call /ardrone/setflightanimation 14 0")
					self.color3 = 0
					
				elif (currentR > (color4R - 10) and currentR < (color4R + 10)) and (currentG > (color4G - 10) and currentG < (color4G + 10)) and (currentB > (color4B - 10) and currentB < (color4B + 10)):
					self.statusMessage = "color4 seen again"
					#os.system("rosservice call /ardrone/setledanimation 3 4 5")
					os.system("rosservice call /ardrone/setflightanimation 17 0")
					self.color4 = 0
			
		finally:
			self.imageLock.release()

	def ReceiveNavdata(self,navdata):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# Update the message to be displayed
		msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
		#self.statusMessage = '{} (Battery: {}%)'.format(msg,int(navdata.batteryPercent))
		
		self.tagLock.acquire()
		try:
			if navdata.tags_count > 0:
				self.tags = [(navdata.tags_xc[i],navdata.tags_yc[i],navdata.tags_distance[i]) for i in range(0,navdata.tags_count)]
			else:
				self.tags = []
		finally:
			self.tagLock.release()

if __name__=='__main__':
	import sys
	rospy.init_node('ardrone_video_display')
	app = QtGui.QApplication(sys.argv)
	display = DroneVideoDisplay()
	display.show()
	status = app.exec_()
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
