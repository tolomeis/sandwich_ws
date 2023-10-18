#!/usr/bin/env python
import rospy

from std_srvs.srv import Empty
from sandwich.srv import LaunchNode, LaunchNodeResponse, StopNode, StopNodeResponse
import roslaunch
import subprocess

def launcher(req):
	package = req.package_name
	node_file = req.file_name
	ret = True
	try:
		if node_file.endswith('.launch'):
			subprocess.Popen(['roslaunch {0} {1}'.format(package,node_file)], shell=True)
		else:
			subprocess.Popen(['rosrun {0} {1}'.format(package, node_file)], shell=True)
		ret = False
	except:
		ret = True
	return LaunchNodeResponse(ret)

def nodeKill(req):
	ret = True
	try:
		subprocess.Popen(['rosnode kill {0}'.format(req.node_name)], shell=True)
		ret = False
	except:
		ret = True
	return StopNodeResponse(ret)


def start():
	rospy.init_node('node_manager')
	start_sr = rospy.Service('launch_node', LaunchNode, launcher)
	stop_sr = rospy.Service('stop_node', StopNode, nodeKill)
	# starts the node
	rospy.spin()

if __name__ == '__main__':
	start()
