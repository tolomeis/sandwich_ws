#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import serial
import struct
import Jetson.GPIO as GPIO
import numpy as np

from rospy.core import NullHandler

# Parameters to scale the motors speed.
lin_gain = 100
ang_gain = 100
max_speed = 50
dead_zone = 12

# Connection to the RPI Pico
pico = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)


def callback(velMsg):
	# Compute the desired speed for each motor, using the gains
	vdx = (velMsg.linear.x)*lin_gain		# Using the linear velocity
	vsx = vdx
	vdx -= velMsg.angular.z*ang_gain		# And the angular velocity
	vsx += velMsg.angular.z*ang_gain
	
	vsx = vsx + np.sign(vsx)*dead_zone				# Add offset to compensate dead zone
	vdx = vdx + np.sign(vdx)*dead_zone

	vsx = min(max_speed, max(-max_speed, vsx))			#Constrain the values between 100% and -100%
	vdx = min(max_speed, max(-max_speed, vdx))	
	#rospy.loginfo("requested vdx=%d, vsx=%d", vdx, vsx)
	
	pico.write('s'.encode()) 				# 's' is the start character for code running in the PICO
	pico.write(struct.pack("b",vdx))		# Sends left and right speed, encoding in 1 byte signed
	pico.write(struct.pack("b",-vsx))		# (send minus for wiring compatibility)


def stop():
	pico.write('s'.encode()) 			# 's' is the start character for code running in the PICO
	pico.write(struct.pack("b",0))		# Sends left and right speed, encoding in 1 byte signed
	pico.write(struct.pack("b",0))

def debugPICO(event):
	while pico.readable():
		rospy.loginfo(pico.readline())


if __name__ == '__main__':
	# Use GPIO 7 to signal the PICO to start driving the motors
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(7, GPIO.OUT, initial=GPIO.LOW)

	rospy.init_node('motcontrol', anonymous=False)		# ROS node init
	lin_gain = rospy.get_param('~lin_gain', 50)			# parameters init
	ang_gain = rospy.get_param('~ang_gain', 50)
	rospy.loginfo("%s is %s", rospy.resolve_name('~lin_gain'), lin_gain)	# log parameters value, using the resolve_name trick
	rospy.loginfo("%s is %s", rospy.resolve_name('~ang_gain'), ang_gain) 
	rospy.Subscriber("cmd_vel", Twist, callback)		# subscribe to topic

	rospy.on_shutdown(stop)
	rospy.loginfo("motcontrol started") 
	
	GPIO.output(7,GPIO.HIGH)

	rospy.Timer(rospy.Duration(0.1), debugPICO)
	rospy.spin()
