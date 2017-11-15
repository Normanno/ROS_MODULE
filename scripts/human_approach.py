#!/usr/bin/env python

import rospy
import sys
import os
import time
from math import pow
from datetime import datetime

from xml.etree import ElementTree as ET
from base_class import topics as base_topics
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_srvs.srv import Empty

from base_class import Base


class HumanApproach(Base):

    def __init__(self, topics, config=''):
        super(HumanApproach, self).__init__()
        self.topics = topics
        self.odometry_subscriber = rospy.Subscriber(self.topics["subscribers"]["odometry"],
                                                    MarkerArray, self.process_odometry, queue_size=1)
        self.velocity_ctrl_subscriber = rospy.Subscriber(topics["subscribers"]["velocity_ctrl"],
                                                         Twist, self.process_velocity, queue_size=10)
        self.motors_subscriber = rospy.Subscriber(topics["subscribers"]["motors"],
                                                  Bool, self.process_motors, queue_size=10)
        self.velocity_publisher = rospy.Publisher(self.topics["publishers"]["velocity"],
                                                  Twist, queue_size=1)
        self.human_reached_publisher = rospy.Publisher(self.topics["publishers"]["human_reached"],
                                                       Bool, queue_size=10)
        self.smartband_state_subscriber = rospy.Subscriber(topics["publishers"]["smartband_state"],
                                                         Bool, self.process_smartband_state, queue_size=1)
        self.stop_distance_subscriber = rospy.Subscriber(topics["publishers"]["stop_distance"],
                                                         Float32, self.process_stop_distance, queue_size=1)
        self.conf_dir = config
        if config[len(config) - 1] != '/':
            self.conf_dir += '/'
        self.velocity_file_path = self.conf_dir + 'velocity.xml'
        self.stop_distance_file_path = self.conf_dir + 'stop_distance.xml'

        self.humans = 0
        self.human_detected = False
        self.human_reached = True
        self.smartband_connected = False
        self.motors_enabled = self.ros_aria_connected
        self.pose = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.stop_distance = 0.0
        self.stop_distance_velocity_adaptation = 0.0
        self.approach_linear_velocity = 0.0
        self.approach_angular_velocity = 0.0
        self.actual_velocity = Twist()

        self.stop_detected_time = 0.0
        self.null_velocity_time = 0.0

        self.init_parameters()

        rospy.init_node("human_approach", anonymous=True)

    def init_parameters(self):
        if os.path.isfile(self.velocity_file_path):
            velocity_tree = ET.parse(self.velocity_file_path)
            velocity_root = velocity_tree.getroot()
            if velocity_root.find('linear') is not None:
                self.approach_linear_velocity = float(velocity_root.find('linear').text)
            if velocity_root.find('angular') is not None:
                self.approach_angular_velocity = float(velocity_root.find('angular').text)
        else:
            print "Error : Can't find " + self.velocity_file_path + " no such file or directory!"

        if os.path.isfile(self.stop_distance_file_path):
            stop_distance_tree = ET.parse(self.stop_distance_file_path)
            stop_distance_root = stop_distance_tree.getroot()
            if stop_distance_root.find('initial') is not None:
                self.stop_distance = float(stop_distance_root.find('initial').text)
        else:
            print "Error : Can't find " + self.stop_distance_file_path + 'no such file or directory!'

    def process_motors(self, motors_msg):
        self.motors_enabled = motors_msg.data

    def process_smartband_state(self, msg):
        if self.smartband_connected and not msg.data:
            print "** Smartband : disconnected **"
        elif not self.smartband_connected and msg.data:
            print "** Smartband : connected **"
        self.smartband_connected = msg.data

    def process_stop_distance(self, msg):
        if self.stop_distance != msg.data:
            print 'stop_distance ' + str(self.stop_distance) + '->' + str(msg.data)
        self.stop_distance = msg.data

    def process_velocity(self, velocity_msg):
        #TODO take this value with dynamic_reconfigure
        decelleration = 0.5
        self.approach_linear_velocity = velocity_msg.linear.x
        self.approach_angular_velocity = velocity_msg.angular.z
        # distance traveled by the robot in 0.1 seconds (RosAria update frequency)
        distance = self.approach_linear_velocity * 0.15
        # Stop distance adaptation sum of :
        # - uniform decelleration distance to stop = v^2 / 2*decelleration
        # - robot walkable distance for late frame
        self.stop_distance_velocity_adaptation = pow(self.approach_linear_velocity, 2) / (2 * decelleration)
        self.stop_distance_velocity_adaptation += 2 * distance
        if not self.human_reached:
            self.actual_velocity.linear.x = self.approach_linear_velocity
            self.velocity_publisher.publish(self.actual_velocity)

    def process_odometry(self, odometry):
        if self.humans < len(odometry.markers):
            print "++ humans detected: " + str(self.humans) + "->" + str(len(odometry.markers)) + " ++"
        elif self.humans > len(odometry.markers):
            print "-- humans detected: " + str(self.humans) + "->" + str(len(odometry.markers)) + " --"

        self.humans = len(odometry.markers)
        if self.humans >= 1:
            self.human_detected = True
            # OpenPtrack publish distance in feet, we use meters
            # Axis as in rep 103, x->forward y->left z->up
            self.pose["x"] = odometry.markers[0].pose.position.x
            self.pose["y"] = odometry.markers[0].pose.position.y
            self.pose["z"] = odometry.markers[0].pose.position.z
        else:
            self.human_detected = False
            self.pose["x"] = 0.0
            self.pose["y"] = 0.0
            self.pose["z"] = 0.0

    def publish_human_reached(self):
        new_msg = Bool()
        new_msg.data = self.human_reached
        self.human_reached_publisher.publish(new_msg)

    def near_human(self):
        return self.pose["x"] <= (self.stop_distance + self.stop_distance_velocity_adaptation)

    def approach(self):
        frequency = 60
        rate = rospy.Rate(frequency)
        velocity_update = False
        stop = False
        counter = 0
        print "** Smartband : disconnected **"
        while not rospy.is_shutdown():
            if self.smartband_connected and self.human_detected:
                stop = False
                if counter == 10:
                    counter = 0
                counter += 1
                # ROSAria mantain the velocity
                # move toward human
                if not self.near_human():
                    if self.human_reached:
                        print "**** Human detected: MOVING ****"
                        self.human_reached = False
                        self.actual_velocity.linear.x = self.approach_linear_velocity
                        velocity_update = True
                elif self.near_human():
                    print "not near"
                    if not self.human_reached:
                        print "**** Human detected: REACHED ****"
                        stop_distance_log_msg = "x " + str(self.pose["x"]) + " - sd " + str(self.stop_distance) + " - " + str(self.stop_distance_velocity_adaptation)
                        rospy.loginfo("TIME [" + str() + "]" + stop_distance_log_msg)
                        self.human_reached = True
                        self.actual_velocity.linear.x = 0.0
                        velocity_update = True
                # turn toward human
                new_angular_velocity = min(1.0, self.pose["y"]) * self.approach_angular_velocity
                if new_angular_velocity != self.actual_velocity.angular.z:
                    self.actual_velocity.angular.z = new_angular_velocity
                    velocity_update = True
            elif self.smartband_connected and not self.human_detected:
                stop = False
                # turn around looking for a human
                self.actual_velocity.linear.x = 0.0
                self.actual_velocity.angular.z = self.approach_angular_velocity
                velocity_update = True
            # if smartband is not connected or motors aren't enables
            elif not self.smartband_connected and not stop:
                self.actual_velocity.angular.z = 0.0
                self.actual_velocity.linear.x = 0.0
                velocity_update = True
                stop = True

            if velocity_update:
                self.velocity_publisher.publish(self.actual_velocity)
                velocity_update = False

            self.publish_human_reached()
            rate.sleep()


if __name__ == "__main__":
    try:
        config_dir = sys.argv[1]
        ha = HumanApproach(base_topics, config_dir)
        ha.approach()
    except rospy.ROSInterruptException:
        print "aproach interrupt exception"
