#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from tf.transformations import euler_from_quaternion, quaternion_matrix
import numpy as np

err_int = 0
err = 0
found = False
linvel = 0.0
fwd_err = 0.0
fwd_speed = 0.0

fwd_err_int = 0.0

DELTA_T = 0.06
TARGET_DIST = 0
TAG_ID = 2

K1 = 0.6
K2 = 1
LAMBDA = 1
def callback(out):
	global err
	global found
	global size
	for tag in out.detections:
		if tag.id[0] == TAG_ID:
			twist_msg = Twist()
			# Pose in camera frame
			pose_list = tag.pose.pose.pose.position
			tag_pose = np.matrix([pose_list.x,pose_list.y, pose_list.z]).T
			cam2wheel= np.matrix([0,0.04,-0.145]).T
			tag_pose += cam2wheel
			tag_pose[1] = 0

			
			# Orientation between camera frame and tag
			q = tag.pose.pose.pose.orientation
			q_list = [q.x, q.y, q.z, q.w]
			(roll, pitch, yaw) = euler_from_quaternion(q_list)
			Rz = np.matrix([[np.cos(yaw), -np.sin(yaw), 0],[np.sin(yaw), np.cos(yaw), 0], [0,0,1]])
			Ry = np.matrix([[np.cos(pitch), 0, np.sin(pitch)], [0,1,0],[-np.sin(pitch), 0, np.cos(pitch)]])
			Rx = np.matrix([[1,0,0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
			R = (Rz*Ry)*Rx
			goal_vect = np.dot(R,np.array([0,0,-1])).T
			goal_vect[1] = 0
			theta = np.arctan2(-goal_vect[0],goal_vect[2])

			tag_offset = np.array([0,0,0.25])
			print(tag_pose)
			print(tag_offset)
			print(np.dot(R,tag_offset))
			#tag_pose += np.dot(R,tag_offset)
			
			
			rho = np.linalg.norm(tag_pose,2)

			alpha = np.arctan2(-tag_pose[0], tag_pose[2])
			
			psi = alpha - theta

			# LYAPUNOV CONTROL
			fwd_speed = K1*rho*np.cos(alpha)
			ang_speed = K1*np.cos(alpha)*np.sin(alpha)*(1- psi/(LAMBDA*alpha)) + alpha*K2/LAMBDA
			#print([rho,alpha,psi])
			print([fwd_speed, ang_speed])
			

			if rho < 0.05:
				fwd_speed = 0.0
				ang_speed = 0.0
			fwd_speed = max(-0.3,min(0.3,fwd_speed))
			ang_speed = max(-0.2, min(0.2,ang_speed))

			twist_msg.linear.x = fwd_speed
			twist_msg.angular.z = ang_speed
			pub.publish(twist_msg)
			
			found = False



if __name__ == '__main__':
	#init ROS node
	rospy.init_node('apriltag_follow', anonymous=False)

	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)
	pub = rospy.Publisher("cmd_vel",Twist,queue_size=5)

	rospy.loginfo("started")
	rospy.spin()
