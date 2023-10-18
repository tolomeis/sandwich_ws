#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty
import os
import roslaunch
import subprocess

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 3 aka left stick vertical controls linear speed
# axis 2 aka left stick horizonal controls angular speed
oba_active = False

proc = []

def callback(data):
	L_GAIN = 0.3
	A_GAIN = 0.3
	twist = Twist()
	L_GAIN = 0.5 + (1 - data.axes[5])*0.35
	if data.buttons[5] == 1:
		A_GAIN = 0.0
		L_GAIN = 0.0
	twist.linear.x = L_GAIN*data.axes[4]
	twist.angular.z = A_GAIN*data.axes[3]
	pub.publish(twist)

	if data.buttons[0] == 1 and data.buttons[4] == 1:
		restart_motcontrol()
		return

	if data.buttons[4] == 1:
		trigger_estop()
		return

	if data.buttons[7] == 1:
		start_oba()

	if data.buttons[8] == 1 and data.buttons[6] == 1:
		rospy.logwarn("SHUTTING DOWN")
		subprocess.Popen(['shutdown now'],shell=True)



def trigger_estop():
	rospy.wait_for_service('stop_motors')
	try:
		stopsrv = rospy.ServiceProxy('stop_motors', Empty)
		stopsrv()
	except rospy.ServiceException as e:
		rospy.logwarn("ESTOP call failed: %s"%e)


def restart_motcontrol():
	rospy.wait_for_service('clear_estop')
	try:
		clear = rospy.ServiceProxy('clear_estop', Empty)
		clear()
	except rospy.ServiceException as e:
		rospy.logwarn("CLEAR call failed: %s"%e)

def start_oba():
	global oba_active, proc
	if oba_active == False:
		# launch = roslaunch.parent.ROSLaunchParent(uuid, [oba_path])
		# launch.start()
		# rospy.loginfo("OBA started")
		proc = subprocess.Popen(['roslaunch sandwich_autonomous sandwich_auto.launch'],shell=True)
		oba_active = True
	else:
		# launch.shutdown()
		proc.kill()
		subprocess.Popen(['rosnode kill segnet cheese_ad'],shell=True)
		oba_active = False	



# Intializes everything
def start():

	global pub, uuid

	rospy.init_node('Joy4Sandwich')
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)

	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	# subscribed to joystick inputs on topic "joy"
	rospy.Subscriber("joy", Joy, callback)
	# starts the node
	rospy.spin()

if __name__ == '__main__':
	start()
