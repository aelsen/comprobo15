#!/usr/bin/env python

"""
Robot Localization Project
Computational Robotics 2015
Thomas Nattestad
Antonia Elsen
"""

import rospy

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap
from copy import deepcopy

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

import math
import time
import random

import numpy as np
from numpy.random import random_sample
from sklearn.neighbors import NearestNeighbors
from occupancy_field import OccupancyField

from helper_functions import (convert_pose_inverse_transform,
                              convert_translation_rotation_to_pose,
                              convert_pose_to_xy_and_theta,
                              angle_diff)

class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self,x=0.0,y=0.0,theta=0.0,w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """ 
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
        return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))

    # TODO: define additional helper functions if needed

class ParticleFilter:
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            initialized: a Boolean flag to communicate to other class methods that initializaiton is complete
            base_frame: the name of the robot base coordinate frame (should be "base_link" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            laser_max_distance: the maximum distance to an obstacle we should use in a likelihood calculation
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            laser_subscriber: listens for new scan data on topic self.scan_topic
            tf_listener: listener for coordinate transforms
            tf_broadcaster: broadcaster for coordinate transforms
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            map: the map we will be localizing ourselves in.  The map should be of type nav_msgs/OccupancyGrid

            particle_variance: The meter amount that the particle cloud can vary from center by 
    """
    def __init__(self):
        self.initialized = False        # make sure we don't perform updates before everything is setup
        rospy.init_node('pf')           # tell roscore that we are creating a new node named "pf"

        self.base_frame = "base_link"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from 

        self.n_particles = 300          # the number of particles to use

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update

        self.laser_max_distance = 2.0   # maximum penalty to assess in the likelihood field model

        self.particle_distance_variance = .3        # Meter
        self.particle_angle_variance = 3.141 / 3  # Radians

        self.sigma = .5
        self.reselection_amount = 0.05      
        self.most_likely_particle = None

        # Setup pubs and subs

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.pose_listener = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)
        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        self.survivor_pub = rospy.Publisher("survivorcloud", PoseArray, queue_size=10)

        # laser_subscriber listens for data from the lidar
        self.laser_subscriber = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_received)

        # enable listening for and broadcasting coordinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        self.particle_cloud = []
        self.survivor_cloud = []

        self.current_odom_xy_theta = []

        # request the map from the map server, the map should be of type nav_msgs/OccupancyGrid
        rospy.wait_for_service('static_map')
        try:
          static_map_svc = rospy.ServiceProxy("static_map", GetMap)
        except rospy.ServiceException, e:
          print "Service call failed: %s"%e
        static_map = static_map_svc()
        self.occupancy_field = OccupancyField(static_map.map)

        # for now we have commented out the occupancy field initialization until you can successfully fetch the map
        #self.occupancy_field = OccupancyField(map)
        self.initialized = True

    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            There are two logical methods for this:
                (1): compute the mean pose
                (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # First make sure that the particle weights are normalized
        self.normalize_particles()

        # ones = []
        # Calculate the average pos of all of the likeliest particles
        # for aParticle in self.particle_cloud:     
        #   if aParticle.w == 1.0:      
        #     ones.append(aParticle)      
                    
        # # Average their coordinates
                # print "sum: ", self.sum_particles(ones)
                # print "most likely: ", self.most_likely_particle.as_pose()
        # self.robot_pose = self.sum_particles(ones)       

        # Deprecated -- assign new robot pose using most likely particle    
        if self.most_likely_particle is not None:     
            self.robot_pose = self.most_likely_particle.as_pose()
            print "updating robot pose to", self.most_likely_particle.as_pose()
        else:
            print "ERR: NO LIKELY PARTICLE. NOT UPDATING POSE"

                
        # Deprecated -- Don't update pose
        # self.robot_pose = Pose()

    def update_particles_with_odom(self, msg):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.

            msg: this is not really needed to implement this, but is here just in case.
        """
        # print "Odom particle updating"

        new_odom_xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        # Determine angular rotation and displacement from odom history
        r1 = math.atan2(delta[1], delta[0]) - old_odom_xy_theta[2]
        d = math.sqrt((delta[0] ** 2) + (delta[1]**2))                 
        r2 = delta[2] - r1
        
        # Update particle cloud
        for particle in self.particle_cloud:
            particle.theta = particle.theta + r1           # Initial rotation           
            odom_x = math.cos(particle.theta) * d         # Add displacement
            odom_y = math.sin(particle.theta) * d
            particle.x = particle.x + odom_x
            particle.y = particle.y + odom_y
            particle.theta = particle.theta + r2           # Second rotation
            
        #noise it up some:
        self.update_particles_with_noise()
        # For added difficulty: Implement sample_motion_odometry (Prob Rob p 136)

    def map_calc_range(self,x,y,theta):
        """ Difficulty Level 3: implement a ray tracing likelihood model... Let me know if you are interested """
        # TODO: nothing unless you want to try this alternate likelihood model
        pass

    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample.
        """
        # make sure the distribution is normalized
        self.normalize_particles()
        # print [x.w for x in self.particle_cloud]

        # #Get the top n particles 
        self.survivor_cloud = self.get_top_particles(int(self.n_particles * self.reselection_amount))
 
        # create new children of the survivors with variance, eliminate non-survivors       
        self.particle_cloud = []  
        # print "Printing survivors:"   
        for aSurvivor in self.survivor_cloud:
            #print "survivor " + str(s) +": ", aSurvivor.as_pose()
            self.particle_cloud.append(aSurvivor)     
            for i in range(1, int(1/self.reselection_amount)):      
                x_hyp = aSurvivor.x + (random.random() - 0.5) * self.particle_distance_variance       
                y_hyp = aSurvivor.y + (random.random() - 0.5) * self.particle_distance_variance       
                theta_hyp = aSurvivor.theta + (random.random() - 0.5) * self.particle_angle_variance      
        
                self.particle_cloud.append(Particle(x_hyp, y_hyp, theta_hyp))
        self.normalize_particles()
        # # OLD: New approach: just close the most likely
        # self.particle_cloud = []
        # self.particle_cloud.append(self.most_likely_particle)
        # for i in range(1, self.n_particles):
        #     x_hyp = self.most_likely_particle.x + (random.random() - 0.5) * self.particle_distance_variance       
        #     y_hyp = self.most_likely_particle.y + (random.random() - 0.5) * self.particle_distance_variance       
        #     theta_hyp = self.most_likely_particle.theta + (random.random() - 0.5) * self.particle_angle_variance      
    
        #     self.particle_cloud.append(Particle(x_hyp, y_hyp, theta_hyp))
        
        
        # # OLD: we tried using draw random sample with little success 
        # # print "weights: ", self.particle_weights
        # survivors = self.draw_random_sample(self.particle_cloud, self.particle_weights,         
        #                             self.reselection_amount * self.n_particles)    
        # self.survivor_cloud = survivors
        # # print "Num survivors / all: ", len(survivors), " / ", len(self.particle_cloud)      
        
                
        # # create new children of the survivors with variance, eliminate non-survivors       
        # self.particle_cloud = []  
        # # print "Printing survivors:"   
        # s = 0
        # for aSurvivor in survivors:
        #     #print "survivor " + str(s) +": ", aSurvivor.as_pose()
        #     s += 1
        #     self.particle_cloud.append(aSurvivor)     
        #     for i in range(1, int(1/self.reselection_amount)):      
        #         x_hyp = aSurvivor.x + (random.random() - 0.5) * self.particle_distance_variance       
        #         y_hyp = aSurvivor.y + (random.random() - 0.5) * self.particle_distance_variance       
        #         theta_hyp = aSurvivor.theta + (random.random() - 0.5) * self.particle_angle_variance      
        
        #         self.particle_cloud.append(Particle(x_hyp, y_hyp, theta_hyp))

    def get_top_particles(self, amount):
        # print [x.w for x in self.particle_cloud]
        return sorted(self.particle_cloud, key = lambda x: x.w, reverse=True)[0:amount]

    def update_particles_with_laser(self, msg):
        """ Updates the particle weights in response to the scan contained in the msg """
        # Determine weights using particle-lidar-map likelihood
        # print "updating with laser"     
        for aParticle in self.particle_cloud:       
            # Iterate over lidar data (distances) by angle (degrees)
            likelihoods = [0] * 360         # Stores likelihood of each lidar slice       
            
            for angle_degrees in range(0, 360):       
                    
                # Get distance measure from robot at the given angle
                distance = msg.ranges[angle_degrees]        # Raw lidar at lidar angle      
            
                angle_rad = float(angle_degrees) * math.pi / 180   # Convert to rads       
                angle_sum = aParticle.theta + angle_rad     
                
                #
                hyp_x = aParticle.x + (math.cos(angle_sum) * distance)      
                hyp_y = aParticle.y + (math.sin(angle_sum) * distance)      
            
                d = self.occupancy_field.get_closest_obstacle_distance(hyp_x, hyp_y)        
                        
                likelihoods[angle_degrees] = math.exp( (-d**2) / (self.sigma**2) )      
                        
            # Combine all likelihoods to calculate particle weight      
                # average the cube of each likelihood       
            aParticle.w = math.fsum([l ** 3 for l in likelihoods]) / len(likelihoods)       
                
        self.normalize_particles()   
                
        # debug: count to make sure only one particle got a weight of 1
        # normalized_counter = 0      
        # for index, aParticle in enumerate(self.particle_cloud):     
        #     if aParticle.w == 1.0:      
        #       normalized_counter += 1       
                    
        # print "P of weight 1.0:", normalized_counter

    @staticmethod
    def weighted_values(values, probabilities, size):
        """ Return a random sample of size elements from the set values with the specified probabilities
            values: the values to sample from (numpy.ndarray)
            probabilities: the probability of selecting each element in values (numpy.ndarray)
            size: the number of samples
        """
        bins = np.add.accumulate(probabilities)
        return values[np.digitize(random_sample(size), bins)]

    @staticmethod
    def draw_random_sample(choices, probabilities, n):
        """ Return a random sample of n elements from the set choices with the specified probabilities
            choices: the values to sample from represented as a list
            probabilities: the probability of selecting each element in choices represented as a list
            n: the number of samples
        """
        values = np.array(range(len(choices)))
        probs = np.array(probabilities)
        bins = np.add.accumulate(probs)
        inds = values[np.digitize(random_sample(n), bins)]
        samples = []
        for i in inds:
            samples.append(deepcopy(choices[int(i)]))
        return samples

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(xy_theta)
        self.fix_map_to_odom_transform(msg)

    def update_particles_with_noise(self):
        for aParticle in self.particle_cloud:
            aParticle.x = aParticle.x + (random.random() - 0.5) * self.particle_distance_variance 
            aParticle.y = aParticle.y + (random.random() - 0.5) * self.particle_distance_variance
            aParticle.theta = aParticle.theta + (random.random() - 0.5) * self.particle_angle_variance 


    def get_particles_with_noise(self, xy_theta):
        self.particle_cloud = []
        
        for i in range(0, self.n_particles):
            x_hyp = xy_theta[0] + (random.random() - 0.5) * self.particle_distance_variance 
            y_hyp = xy_theta[1] + (random.random() - 0.5) * self.particle_distance_variance
            theta_hyp = xy_theta[2] + (random.random() - 0.5) * self.particle_angle_variance 
 
            self.particle_cloud.append(Particle(x_hyp, y_hyp, theta_hyp))


    def initialize_particle_cloud(self, xy_theta=None):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is ommitted, the odometry will be used """
        if xy_theta == None:
            xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
        
        self.get_particles_with_noise(xy_theta)

        self.normalize_particles()
        self.robot_pose = self.odom_pose.pose

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        # print "Begin normalizing"

        total = 0
        for aParticle in self.particle_cloud:
            total += aParticle.w

        for aParticle in self.particle_cloud:
            aParticle.w = aParticle.w / total

        self.particle_weights = []
        for aParticle in self.particle_cloud:
            self.particle_weights.append(aParticle.w)

        # print "sum:", sum(self.particle_weights)

        # Find your mom (largest weight) again, just because
        heaviest = 0
        for aParticle in self.particle_cloud:
            if aParticle.w > heaviest:
                heaviest = aParticle.w
                self.most_likely_particle = aParticle
        
    def publish_particles(self, msg):
        particles_conv = []
        for p in self.particle_cloud:   
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.map_frame),
                                  poses=particles_conv))
        # Debug               
        survivor_conv = []
        for p in self.survivor_cloud:   
            survivor_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.survivor_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.map_frame),
                                  poses=survivor_conv))

    def scan_received(self, msg):
        """ This is the default logic for what to do when processing scan data.
            Feel free to modify this, however, I hope it will provide a good
            guide.  The input msg is an object of type sensor_msgs/LaserScan """
        if not(self.initialized):
            # wait for initialization to complete
            return

        if not(self.tf_listener.canTransform(self.base_frame,msg.header.frame_id,msg.header.stamp)):
            # need to know how to transform the laser to the base frame
            # this will be given by either Gazebo or neato_node
            return

        if not(self.tf_listener.canTransform(self.base_frame,self.odom_frame,msg.header.stamp)):
            # need to know how to transform between base and odometric frames
            # this will eventually be published by either Gazebo or neato_node
            return

        # calculate pose of laser relative ot the robot base
        p = PoseStamped(header=Header(stamp=rospy.Time(0),
                                      frame_id=msg.header.frame_id))
        self.laser_pose = self.tf_listener.transformPose(self.base_frame,p)

        # find out where the robot thinks it is based on its odometry
        p = PoseStamped(header=Header(stamp=msg.header.stamp,
                                      frame_id=self.base_frame),
                        pose=Pose())
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)
        # store the the odometry pose in a more convenient format (x,y,theta)
        new_odom_xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)

        if not(self.particle_cloud):
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud()
            # cache the last odometric pose so we can only update our particle filter if we move more than self.d_thresh or self.a_thresh
            self.current_odom_xy_theta = new_odom_xy_theta
            # update our map to odom transform now that the particles are initialized
            self.fix_map_to_odom_transform(msg)
        elif (math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh):
            # we have moved far enough to do an update!
            self.update_particles_with_odom(msg)    # update based on odometry
            self.update_particles_with_laser(msg)   # update based on laser scan
            self.update_robot_pose()                # update robot's pose
            self.resample_particles()               # resample particles to focus on areas of high density
            self.fix_map_to_odom_transform(msg)     # update map to odom transform now that we have new particles
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg)

    def fix_map_to_odom_transform(self, msg):
        """ This method constantly updates the offset of the map and 
            odometry coordinate systems based on the latest results from
            the localizer """
        (translation, rotation) = convert_pose_inverse_transform(self.robot_pose)
        p = PoseStamped(pose=convert_translation_rotation_to_pose(translation,rotation),
                        header=Header(stamp=msg.header.stamp,frame_id=self.base_frame))
        self.odom_to_map = self.tf_listener.transformPose(self.odom_frame, p)
        (self.translation, self.rotation) = convert_pose_inverse_transform(self.odom_to_map.pose)

    def broadcast_last_transform(self):
        """ Make sure that we are always broadcasting the last map
            to odom transformation.  This is necessary so things like
            move_base can work properly. """
        if not(hasattr(self,'translation') and hasattr(self,'rotation')):
            return
        self.tf_broadcaster.sendTransform(self.translation,
                                          self.rotation,
                                          rospy.get_rostime(),
                                          self.odom_frame,
                                          self.map_frame)

if __name__ == '__main__':
    n = ParticleFilter()
    r = rospy.Rate(5)

    while not(rospy.is_shutdown()):
        # in the main loop all we do is continuously broadcast the latest map to odom transform
        n.broadcast_last_transform()
        r.sleep()