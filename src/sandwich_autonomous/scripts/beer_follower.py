#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
'''
err_int = 0
err = 0
f = False
linvel = 0.0
fwd_err = 0.0
fwd_speed = 0.0

def callback(out):
	global err_int
	global err
	global f
	global fwd_err
	BEER_ID = 0

	for detection in out.detections:
		for res in detection.results:
			if res.id == BEER_ID:
				f = True
				x = detection.bbox.center.x
				
				y = detection.bbox.center.y
				w = 1280
				h = 720
				err = (x - (w/2))/(w/2)
				fwd_err = (600 - detection.bbox.size_y)/w
				#fwd_err = 0.3 -  abs(detection.bbox.size_y - w)/w 
				rospy.loginfo("BIRRAAAAAA in %s e distanza %s", x, fwd_err) 

# Controllo PI
def carrello(event):
	global err
	global err_int
	global f
	global linvel
	global fwd_speed
	twist_msg = Twist()
	if f == True:
		Kp = 0.5
		Ki = 0
		err_int = err_int + err*0.06
		angvel = - Kp*err - Ki*err_int
		#linvel = linvel + 0.09
		linvel = fwd_err*3
		angvel =  min(1, max(-1, angvel))
		twist_msg.angular.z = angvel
	else:
		twist_msg.angular.z = 0.0
		linvel = linvel*0.7
	#IIR su velocita' di avanzamento
	linvel =  min(0.8, max(-0.8, linvel))
	alpha = 0.5
	fwd_speed  = alpha*fwd_speed + (1-alpha)*linvel 

	twist_msg.linear.x = fwd_speed
	pub.publish(twist_msg)
	f = False

def resettatutto(event):
	global err_int
	err_int = 0

if __name__ == '__main__':
	#init ROS node
	rospy.init_node('beer_follower', anonymous=False)

	rospy.Subscriber("/detectnet/detections", Detection2DArray, callback)
	pub = rospy.Publisher("cmd_vel",Twist,queue_size=5)

	rospy.loginfo("started")
	rospy.Timer(rospy.Duration(0.06), carrello)
	rospy.Timer(rospy.Duration(0.06*15), resettatutto)
	rospy.spin()
'''



err_int = 0
err = 0
found = False
linvel = 0.0
fwd_err = 0.0
fwd_speed = 0.0

fwd_err_int = 0.0
size = 0.0
DELTA_T = 0.06
TARGET_SIZE = 0.8

def callback(out):
	global err
	global found
	global size
	
	BEER_ID = 0
	size_alpha = 0.6

	for detection in out.detections:
		for res in detection.results:
			if res.id == BEER_ID:
				found = True
				x = detection.bbox.center.x
				w = 1280
				h = 720

				err = (x - (w/2))/(w/2)
				size = size_alpha*size + (1-size_alpha)*detection.bbox.size_y/h
				rospy.loginfo("BIRRAAAAAA in %s e grandezza %s", x, size) 

# Controllo PI
def carrello(event):
	# Posizione orizzontale
	global err
	global err_int
	global found
	Kp = 0.5
	Ki = 0

	# Avanzamento:
	global fwd_err_int
	global fwd_speed
	global size	
	Kp_s = 0.8
	Ki_s = 0.2

	twist_msg = Twist()
	if found == True:
		
		err_int = err_int + err*DELTA_T
		angvel = - Kp*err - Ki*err_int
		angvel =  min(1, max(-1, angvel))
		twist_msg.angular.z = angvel

		fwd_err = TARGET_SIZE - size
		fwd_err_int = fwd_err_int + fwd_err*DELTA_T
		fwd_speed = fwd_err*Kp_s + fwd_err_int*Ki_s
	else:
		twist_msg.angular.z = 0.0
		fwd_speed = fwd_speed*0.7

	twist_msg.linear.x = fwd_speed
	pub.publish(twist_msg)
	found = False

def resettatutto(event):
	global err_int
	global fwd_err_int
	err_int = 0
	fwd_err_int = 0


if __name__ == '__main__':
	#init ROS node
	rospy.init_node('beer_follower', anonymous=False)

	rospy.Subscriber("/detectnet/detections", Detection2DArray, callback)
	pub = rospy.Publisher("cmd_vel",Twist,queue_size=5)

	rospy.loginfo("started")
	rospy.Timer(rospy.Duration(DELTA_T), carrello)
	rospy.Timer(rospy.Duration(DELTA_T*15), resettatutto)
	rospy.spin()
