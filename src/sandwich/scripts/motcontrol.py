#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import serial
import struct
import Jetson.GPIO as GPIO
import numpy as np

from rospy.core import NullHandler
from rospy.names import valid_name

# Parameters to scale the motors speed.
lin_gain = 100
ang_gain = 100
max_speed = 100	
dead_zone = 7
vlin = 0.0
vang = 0.0

ALPHA = 0.5
vlin_buff = 0.0
vang_buff = 0.0


# Connection to the RPI Pico
pico = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)

def updatevel(velMsg):
	global vlin
	global vang
	vlin = velMsg.linear.x
	vang = velMsg.angular.z


def send2pico(event):
	# Compute the desired speed for each motor, using the gains
	global vlin_buff
	global vang_buff

	vlin_buff = vlin_buff*ALPHA + (1-ALPHA)*vlin
	vang_buff = vang_buff*ALPHA + (1-ALPHA)*vang

	vdx = (vlin_buff)*lin_gain		# Using the linear velocity
	vsx = vdx
	vdx -= vang_buff*ang_gain		# And the angular velocity
	vsx += vang_buff*ang_gain
	
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
	lin_gain = rospy.get_param('~lin_gain', 60)			# parameters init
	ang_gain = rospy.get_param('~ang_gain', 60)
	max_speed = rospy.get_param('~max_speed', 80)
	rospy.loginfo("%s is %s", rospy.resolve_name('~lin_gain'), lin_gain)	# log parameters value, using the resolve_name trick
	rospy.loginfo("%s is %s", rospy.resolve_name('~ang_gain'), ang_gain) 
	rospy.Subscriber("cmd_vel", Twist, updatevel)		# subscribe to topic

	rospy.on_shutdown(stop)
	GPIO.output(7,GPIO.HIGH)
	rospy.Timer(rospy.Duration(0.02), send2pico)
	rospy.Timer(rospy.Duration(0.1), debugPICO)
	
	rospy.loginfo("motcontrol started") 

	rospy.spin()
