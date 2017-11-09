#!/usr/bin/env python

import rospy
import sys
import os
import time
from xml.etree import ElementTree as ET
from base_class import topics as base_topics
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32

from base_class import Base


class HumanApproach(Base):

    def __init__(self, topics, config=''):
        super(HumanApproach, self).__init__()
        self.topics = topics
        self.odometry_subscriber = rospy.Subscriber(self.topics["subscribers"]["odometry"],
                                                    MarkerArray, self.process_odometry, queue_size=10)
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
        self.human_reached = False
        self.smartband_connected = False
        self.motors_enabled = self.ros_aria_connected
        self.pose = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.stop_distance = 0.0
        self.stop_distance_velocty_adaptation = 0.0
        self.approach_linear_velocity = 0.0
        self.approach_angular_velocity = 0.0
        self.actual_velociy = Twist()

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
        self.approach_linear_velocity = velocity_msg.linear.x
        self.approach_angular_velocity = velocity_msg.angular.z
        # Stop distance adaptation based on linear velocity * (vel_update_freq + time_to_publish)
        self.stop_distance_velocty_adaptation = self.approach_linear_velocity * 0.15
        '''
        print 'velocity adaptation '+str(self.stop_distance_velocty_adaptation)
        print 'stop distance ' + str(self.stop_distance)
        print 'stop distance adaptd' + str(self.stop_distance + self.stop_distance_velocty_adaptation)
        '''

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
        #print "x= " + str(self.pose["x"]) + " sd= " + str(self.stop_distance) + " adapt= " + str(self.stop_distance_velocty_adaptation)
        return self.pose["x"] <= (self.stop_distance + self.stop_distance_velocty_adaptation)

    def approach(self):
        rate = rospy.Rate(10)
        state_change = False
        counter = 0
        state_change = False
        print "** Smartband : disconnected **"
        while not rospy.is_shutdown():
            #print str(self.smartband_connected) +'-'+ str(self.motors_enabled) + '-'+ str(self.human_detected)
            if self.smartband_connected and self.motors_enabled and self.human_detected:
                if counter == 10:
                    #print "x " + str(self.pose["x"]) + " - sd " + str(self.stop_distance)
                    counter = 0
                counter += 1

                # ROSAria mantain the velocity
                # move toward human
                if not self.near_human():
                    if self.human_reached:
                        print "**** Human detected: MOVING ****"
                    self.human_reached = False
                    self.actual_velociy.linear.x = self.approach_linear_velocity

                elif self.near_human():
                    if not self.human_reached:
                        print "**** Human detected: REACHED ****"
                        print "x " + str(self.pose["x"]) + " - sd " + str(self.stop_distance)
                    self.human_reached = True
                    self.actual_velociy.linear.x = 0.0
                # turn toward human
                self.actual_velociy.angular.z = min(1.0, self.pose["y"]) * self.approach_angular_velocity
            elif self.smartband_connected and self.motors_enabled and not self.human_detected:
                # turn around looking for a human
                self.actual_velociy.linear.x = 0.0
                self.actual_velociy.angular.z = self.approach_angular_velocity
            # if smartband is not connected or motors aren't enables
            else:
                self.actual_velociy.angular.z = 0.0
                self.actual_velociy.linear.x = 0.0

            if self.motors_enabled:
                self.velocity_publisher.publish(self.actual_velociy)

            self.publish_human_reached()
            rate.sleep()


if __name__ == "__main__":
    try:
        config_dir = sys.argv[1]
        ha = HumanApproach(base_topics, config_dir)
        ha.approach()
    except rospy.ROSInterruptException:
        print "aproach interrupt exception"
