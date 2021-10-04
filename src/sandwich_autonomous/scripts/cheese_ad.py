#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
from cv_bridge import CvBridge
import cv2
import jetson.utils

bridge = CvBridge()
FLOOR_ID = (0, 128, 0)

fwd_speed = 0.0
ang_speed = 0.0
cX = 0
cY = 0
#setY = 620.0
setX = 640.0
setArea = 0.3
w_class = 20
h_class = 16
MAX_ANG_SPEED = 0.4
MAX_FWD_SPEED = 0.4

MAX_FLOOR_H = 6
W_SCALE = 1

def callback(mask_img):
	global fwd_speed 
	global ang_speed
	global cX
	global cY
	w = mask_img.width
	h = mask_img.height
	cv_image = bridge.imgmsg_to_cv2(mask_img, desired_encoding='passthrough')
	setX = w/2
	floor_max_h = h*0.7
	
	floormat = cv2.inRange(cv_image,FLOOR_ID,FLOOR_ID)

	floor_class = cv2.resize(floormat,(w_class,h_class),interpolation=cv2.INTER_AREA)/255
	floormat = floor_class

	# OpenCV: img[x,y] with 
	#	(0,0) 	--> (x)
	#			|
	#			v
	#			(y)
	floormat[0:MAX_FLOOR_H,:] = np.zeros([MAX_FLOOR_H,w_class])
	
	weights = np.flip(np.matrix(range(1,MAX_FLOOR_H+1))*W_SCALE)*np.ones(1,w_class) 

	floormat = np.multiply(floormat,weights)
	M = cv2.moments()
	if M["m00"] != 0:
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
	else:
 		cX = w/2
		cY = 0

	
	fwd_pixels = np.array(range(7,13))
	# Metodo 1: conto quadrati occupati
	fwd_area = np.sum(floor_class[fwd_pixels,:])/(6*9)
	
	# Metodo 2: faccio una sorta di or e produco un vett. colonna. Poi conto
	fwd_vect = np.sum(floor_class[fwd_pixels,:], axis=0)
	fwd_vect = cv2.inRange(fwd_vect,1,255)/255
	fwd_area = np.sum(fwd_vect)/(9)
	
	# cY = cY	
	print([cX,cY, fwd_area])


	fwd_err = -(setArea - fwd_area)
	ang_err = (float(setX) - cX)/float(w)


	fwd_alpha = 0.5
	ang_alpha = 0.3
	Kp_fwd = 8.0
	Kp_ang = 2.0
	Ki_ang = 1.0
	Ki_fwd = 1.0

	fwd_speed = fwd_speed*fwd_alpha + (1-np.abs(ang_speed))*fwd_err*(1.0 - ang_err)*Kp_fwd
	ang_speed = ang_speed*ang_alpha + (1-ang_alpha)*ang_err*Kp_ang
	


	ang_speed = min(MAX_ANG_SPEED, max(-MAX_ANG_SPEED, ang_speed))
	fwd_speed = min(MAX_FWD_SPEED, max(-MAX_FWD_SPEED, fwd_speed))
	
	twist_msg = Twist()
	twist_msg.angular.z = ang_speed
	twist_msg.linear.x = fwd_speed
	pub.publish(twist_msg)


def show_dir(raw_img):
	cv_image = bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')
	w = raw_img.width
	h = raw_img.height

	cv_dir = cv2.circle(cv_image,(cX,cY), 20, (0, 0, 255), 3)
	cv_dir = cv2.line(cv_dir, (w/2,2),(int(w/2 - ang_speed*w/(2*MAX_ANG_SPEED)),2), (0,0,255),4)

	cv_dir = cv2.line(cv_dir, (2,h/2),(2,int(h/2 - fwd_speed*h/(2*MAX_FWD_SPEED))), (0,0,255),4)

	imgpub.publish(bridge.cv2_to_imgmsg(cv_dir, encoding="bgr8"))




def rallenta(event):
	# global fwd_speed 
	# global ang_speed
	# fwd_speed = 0.8*fwd_speed
	# ang_speed = 0.8*ang_speed
	pass

def stop():
	twist_msg = Twist()
	twist_msg.angular.z = 0.0
	twist_msg.linear.x = 0.0
	pub.publish(twist_msg)


if __name__ == '__main__':
	#init ROS node
	rospy.init_node('cheese_autonomous_driving', anonymous=False)
	rospy.Subscriber("segnet/color_mask", Image, callback)
	rospy.Subscriber("camera/image_rect_color", Image, show_dir)

	imgpub = rospy.Publisher("floor",Image, queue_size=5)
	pub = rospy.Publisher("cmd_vel",Twist, queue_size=10)
	rospy.Timer(rospy.Duration(0.2), rallenta)

	rospy.loginfo("started") 
	rospy.on_shutdown(stop)
	rospy.spin()