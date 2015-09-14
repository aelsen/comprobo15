#!/usr/bin/env python

import time
import rospy
import std_msgs.msg 

from neato_node.msg import Bump

def process_bump(data):
	print data

def listener():
	rospy.init_node('emergency_stop', anonymous=True)
	rospy.Subscriber("bump", Bump, process_bump)

	rospy.spin()

if __name__ == '__main__':
	listener()