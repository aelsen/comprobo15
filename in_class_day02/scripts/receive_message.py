#!/usr/bin/env python

""" Exploring publishing topics inside a Python node """

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import rospy

def process_stamped_point(msg):
	print msg.point

rospy.init_node('receive_message')
rospy.Subscriber("/my_point", PointStamped, process_stamped_point)

r = rospy.Rate(10)
while not rospy.is_shutdown():
	r.sleep()