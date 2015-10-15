#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3

speed = 1

class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        self.shown_image = None
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.center_x = 0

        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow('video_window')
        # cv2.setMouseCallback('video_window', self.process_mouse_event)

        # Add configurable sliders
        # RBG
        cv2.namedWindow('threshold_image')
        self.red_lower_bound = 179
        cv2.createTrackbar('red lower bound', 'threshold_image', 179, 255, self.set_red_lower_bound)
        self.red_higher_bound = 255
        cv2.createTrackbar('red higher bound', 'threshold_image', 255, 255, self.set_red_higher_bound)
        # HSV
        self.hue_lower_bound = 0
        cv2.createTrackbar('hue lower bound', 'threshold_image', 0, 255, self.set_hue_lower_bound)
        self.hue_higher_bound = 10
        cv2.createTrackbar('hue higher bound', 'threshold_image', 0, 179, self.set_hue_higher_bound)
        self.sat_lower_bound = 20
        cv2.createTrackbar('sat lower bound', 'threshold_image', 20, 255, self.set_sat_lower_bound)
        self.sat_higher_bound = 255
        cv2.createTrackbar('sat higher bound', 'threshold_image', 255, 255, self.set_sat_higher_bound)
        self.val_lower_bound = 50
        cv2.createTrackbar('val lower bound', 'threshold_image', 50, 255, self.set_val_lower_bound)
        self.val_higher_bound = 255
        cv2.createTrackbar('val higher bound', 'threshold_image', 255, 255, self.set_val_higher_bound)

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        # print self.cv_image.shape

        # cv2.imshow('BGR_video_window', self.cv_image)
        cv2.imshow('HSV_video_window', self.hsv_image)

        cv2.waitKey(5)

        self.binary_isolation_BGR()
        self.binary_isolation_HSV()

        self.compute_center_mass()
        

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        image_info_window = 255*np.ones((500,500,3))
        cv2.putText(image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)


    def binary_isolation_BGR(self):
        # Isolate colors that have high amounts of red
        self.binary_image_BGR = cv2.inRange(self.cv_image, 
                                            (0,0,self.red_lower_bound), 
                                            (255,255, self.red_higher_bound))
        # cv2.imshow('binary_BGR_isolated', self.binary_image_BGR)


    def binary_isolation_HSV(self):
        self.binary_image_HSV = cv2.inRange(self.cv_image, 
                                            (
                                                self.hue_lower_bound, 
                                                self.sat_lower_bound,
                                                self.val_lower_bound), 
                                            (   
                                                self.hue_higher_bound,
                                                self.sat_higher_bound, 
                                                self.val_higher_bound))
        cv2.imshow('binary_HSV_isolated', self.binary_image_HSV)


    def compute_center_mass(self):
        moments = cv2.moments(self.binary_image_HSV)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']

        # Scale center using width of image
        self.center_x = self.center_x / 640.0
        self.center_x = self.center_x - .5
        print "x center:", self.center_x


    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():

            if self.center_x:
                # Issue motor commands based on center of mass of tracked object
                # Go forward if object is near middle of screen
                if math.fabs(self.center_x) < .25:
                    # move forward
                    print "Act: MOVE STRAIGHT"
                    x = 1
                    th = 0
                    pass

                # Turn left if object is to the left
                elif self.center_x < -0.25:
                    print "Act: MOVE LEFT"
                    x = 0
                    th = self.center_x * -1
                    pass

                #Turn right if object is to the right
                elif self.center_x > 0.25:
                    print "Act: NO MOVE RIGHT"
                    x = 0
                    th = self.center_x * -1
                    pass
                else:
                    print "Act: NO MOVE"
                    x = 0
                    th = 0

                # Send motor commands
                twist = Twist()
                twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*speed
                self.pub.publish(twist)

            r.sleep()

    def set_red_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.red_lower_bound = val

    def set_red_higher_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.red_higher_bound = val

    def set_hue_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.hue_lower_bound = val

    def set_hue_higher_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.hue_higher_bound = val

    def set_sat_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.sat_lower_bound = val

    def set_sat_higher_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.sat_higher_bound = val

    def set_val_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.val_lower_bound = val

    def set_val_higher_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.val_higher_bound = val


if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()