#!/usr/bin/env python

import rospy
import os
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

def pubinfo(img):
	# Parse
	timestamp = img.header.stamp
	camera_info_msg = CameraInfo()
	camera_info_msg.header.stamp = timestamp
	camera_info_msg.width = calib_data["image_width"]
	camera_info_msg.height = calib_data["image_height"]
	camera_info_msg.K = calib_data["camera_matrix"]["data"]
	camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
	camera_info_msg.R = calib_data["rectification_matrix"]["data"]
	camera_info_msg.P = calib_data["projection_matrix"]["data"]
	#camera_info_msg.distortion_model = calib_data["distortion_model"]
	camera_info_msg.distortion_model = "plumb_bob"
	pub.publish(camera_info_msg)



if __name__ == '__main__':
		#init ROS node
	rospy.init_node('sw_camera_info', anonymous=False)
	stream = os.popen('rospack find sandwich')
	output = stream.read().replace('\n','')
	filepath = os.path.join(output, "params/ost.yaml")
	file_handle = open(filepath, "r") 
	calib_data = yaml.load(file_handle)
	rospy.Subscriber("camera/image_raw", Image, pubinfo)
	pub = rospy.Publisher("camera/camera_info",CameraInfo,queue_size=5)

	rospy.loginfo("started")
	#rospy.Timer(rospy.Duration(0.05), pubinfo)
	rospy.spin()	