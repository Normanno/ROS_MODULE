#!/usr/bin/env python

import rospy
import sys
import os
from xml.etree import ElementTree as ET
from base_class import topics as base_topics
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from sgr_project.msg import ControlMSG
from base_class import Base


class HumanApproach(Base):

    def __init__(self, topics, config=''):
        super(HumanApproach, self).__init__()
        self.topics = topics
        self.odometry_subscriber = rospy.Subscriber(self.topics["subscribers"]["odometry"],
                                                    MarkerArray, self.process_odometry, queue_size=10)
        self.control_subscriber = rospy.Subscriber(self.topics["subscribers"]["control"],
                                                   ControlMSG, self.process_control, queue_size=10)
        self.velocity_ctrl_subscriber = rospy.Subscriber(topics["subscribers"]["velocity_ctrl"],
                                                         Twist, self.process_velocity, queue_size=10)
        self.velocity_publisher = rospy.Publisher(self.topics["publishers"]["velocity"],
                                                  Twist, queue_size=10)
        self.human_reached_publisher = rospy.Publisher(self.topics["publishers"]["human_reached"],
                                                       Bool, queue_size=10)
        self.conf_dir = config
        if config[len(config) - 1] != '/':
            self.conf_dir += '/'
        self.velocity_file_path = self.conf_dir + 'velocity.xml'
        self.stop_distance_file_path = self.conf_dir + 'stop_distance.xml'

        self.humans = 0
        self.human_detected = False
        self.human_reached = False
        self.smartband_connected = False
        self.motors_enabled = False
        self.pose = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.stop_distance = 0.0
        self.approach_linear_velocity = 0.0
        self.approach_angular_velocity = 0.0
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
            if stop_distance_root.find('stop_distance') is not None:
                self.stop_distance = float(stop_distance_root.find('stop_distance').text)
        else:
            print "Error : Can't find " + self.stop_distance_file_path + 'no such file or directory!'

    def process_control(self, control_msg):
        self.stop_distance = control_msg.stop_distance
        self.smartband_connected = control_msg.smartband_detected
        self.motors_enabled = control_msg.motors_enabled

    def process_velocity(self, velocity_msg):
        self.approach_linear_velocity = velocity_msg.linear.x
        self.approach_angular_velocity = velocity_msg.angular.z
        print 'velocity '+str(velocity_msg)

    def process_odometry(self, odometry):
        if self.humans < len(odometry.markers):
            print "++ humans detected: " + str(self.humans) + "->" + str(len(odometry.markers)) + " ++"
        elif self.humans > len(odometry.markers):
            print "-- humans detected: " + str(self.humans) + "->" + str(len(odometry.markers)) + " --"

        self.humans = len(odometry.markers)
        if self.humans >= 1:
            self.human_detected = True
            #print 'markers '+str(odometry.markers)
            #OpenPtrack publish distance in feet, we use meters
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

    def approach(self):
        velocity = Twist()
        rate = rospy.Rate(10)
        velocity_change = False
        while not rospy.is_shutdown():
            if self.smartband_connected and self.motors_enabled and self.human_detected:
                #move toward human
                if self.pose["x"] > self.stop_distance:
                    if self.human_reached:
                        print "**** Human detected: MOVING ****"
                        self.human_reached = False
                        velocity_change = True
                        #ROSAria mantain the velocity
                    velocity.linear.x = self.approach_linear_velocity
                else:
                    if not self.human_reached:
                        print "**** Human detected: REACHED ****"
                        print "x " + str(self.pose["x"]) + " - sd " + str(self.stop_distance)
                        self.human_reached = True
                        velocity_change = True
                    velocity.linear.x = 0.0
                #turn toward human
                velocity.angular.z = min(1.0, self.pose["y"]) * self.approach_angular_velocity
            elif self.smartband_connected and self.motors_enabled and not self.human_detected:
                velocity.angular.z = self.approach_angular_velocity #turn around to look for a human
            else: #if smartband is not connected
                velocity.angular.z = 0.0
                velocity.linear.x = 0.0
                velocity_change = True

            #Publish messages only if necessary
            if self.motors_enabled and velocity_change:
                self.velocity_publisher.publish(velocity)
                self.publish_human_reached()
                velocity_change = False
            rate.sleep()


if __name__ == "__main__":
    try:
        config_dir = sys.argv[1]
        ha = HumanApproach(base_topics, config_dir)
        ha.approach()
    except rospy.ROSInterruptException:
        print "aproach interrupt exception"
