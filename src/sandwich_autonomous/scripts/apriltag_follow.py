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
	
	BEER_ID = 0
	size_alpha = 0.6
	for tag in out.detections:
		#print(tag.id[0])
		if tag.id[0] == TAG_ID:
			twist_msg = Twist()
			# Pose in camera frame
			pose_list = tag.pose.pose.pose.position
			tag_pose = np.array([pose_list.x,pose_list.y, pose_list.z])
			cam2wheel= np.array([0,0.04,-0.145])
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
			tag_pose += np.dot(R,tag_offset)
			
			rho = np.linalg.norm(tag_pose,2)

			alpha = np.arctan2(-tag_pose[0], tag_pose[2])
			
			psi = alpha - theta

			# LYAPUNOV CONTROL
			fwd_speed = K1*rho*np.cos(alpha)
			ang_speed = K1*np.cos(alpha)*np.sin(alpha)*(1- psi/(LAMBDA*alpha)) + alpha*K2/LAMBDA
			#print([rho,alpha,psi])
			print([fwd_speed, ang_speed])
			'''
			ang_speed = 0
			
			# Pose in sandwich frame (NWU)
			Rsc = np.matrix([[0, -1, 0], [0,0,-1], [1,0,0]])
			tag_nwu = np.dot(Rsc,tag_pose).T
			tag_nwu[2] = 0 						# projection in xy plane

			Rct = quaternion_matrix(q_list)
			Rct = Rct[0:3,0:3]
			# Rot between tag and tag_nwu
			Rttn = np.matrix([[0, -1, 0], [0,0,1], [-1,0,0]])
			# Rot from sandwich to Tag nwu
			Rstn = Rsc*Rct*Rttn
			# Goal direction
			#print(Rstn)
			goal_dir = np.dot(Rstn, np.array([1,0,0])).T
			#print(goal_dir)	
			goal_dir[2,0] = 0.0
			theta = np.arctan2(goal_dir[1,0],goal_dir[0,0])
			
			dts = -np.dot(Rstn,tag_nwu)


			#print(Ry)
			#print([roll,pitch,yaw])
			#dts = - np.dot(Ry,tag_pose)
			#print(dts)

			rho = np.linalg.norm(dts,2)
			
			#@alpha = np.arctan2(dts[0,0],dts[0,2]) + np.pi
			
			#fi = -pitch
			#print(dts)
			fi = np.arctan2(dts[1,0],dts[0,0])
			alpha = fi - theta
			#print([alpha,pitch])	
			fwd_speed = K1*np.cos(alpha)*rho
			ang_speed = K1*np.sin(alpha)*np.cos(alpha)*(alpha + LAMBDA*fi)/alpha + K2*alpha
			'''

			if rho < 0.15:
				fwd_speed = 0.0
				ang_speed = 0.0
			fwd_speed = max(-0.3,min(0.3,fwd_speed))
			ang_speed = max(-0.2, min(0.2,ang_speed))

			twist_msg.linear.x = fwd_speed
			twist_msg.angular.z = ang_speed
			pub.publish(twist_msg)
			
			found = False

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
	rospy.init_node('apriltag_follow', anonymous=False)

	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)
	pub = rospy.Publisher("cmd_vel",Twist,queue_size=5)

	rospy.loginfo("started")
	#rospy.Timer(rospy.Duration(DELTA_T), carrello)
	#rospy.Timer(rospy.Duration(DELTA_T*15), resettatutto)
	rospy.spin()
