#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

#L_GAIN = 0.5
#A_GAIN = 0.5

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 3 aka left stick vertical controls linear speed
# axis 2 aka left stick horizonal controls angular speed
def callback(data):
	L_GAIN = 0.5
	A_GAIN = 0.25
	twist = Twist()
	if data.buttons[4] == 1:
		L_GAIN = 1.0
	if data.buttons[6] == 1:
		A_GAIN = 1.0
	if data.buttons[5] == 1:
		A_GAIN = 0.0
		L_GAIN = 0.0
	twist.linear.x = L_GAIN*data.axes[3]
	twist.angular.z = A_GAIN*data.axes[2]
	pub.publish(twist)

# Intializes everything
def start():
	# publishing to "turtle1/cmd_vel" to control turtle1
	global pub
	pub = rospy.Publisher('cmd_vel', Twist)
	# subscribed to joystick inputs on topic "joy"
	rospy.Subscriber("joy", Joy, callback)
	# starts the node
	rospy.init_node('Joy4Sandwich')
	rospy.spin()

if __name__ == '__main__':
	start()
