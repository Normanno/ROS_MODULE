#!/usr/bin/env python

import rospy
import sys
import os
from xml.etree import ElementTree as ET
from base_class import topics as base_topics
from sgr_project.srv import ComputeStopDistance
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
from sgr_project.msg import SmartbandSensors
from sgr_project.msg import StopDistanceData
from sgr_project.msg import Personality
from base_class import Base


class HumanApproach(Base):

    def __init__(self, topics, config=''):
        super(HumanApproach, self).__init__()
        self.topics = topics
        self.odometry_subscriber = rospy.Subscriber(self.topics["subscribers"]["odometry"], MarkerArray, self.process_odometry)
        self.smartband_subscriber = rospy.Subscriber(topics["subscribers"]["smartband"], SmartbandSensors, self.process_stop_distance)#, 10)
        self.personality_subscriber = rospy.Subscriber(topics["subscribers"]["personality_ctrl"], Personality, self.process_personality)
        self.velocity_ctrl_subscriber = rospy.Subscriber(topics["subscribers"]["velocity_ctrl"], Twist, self.process_velocity)
        self.velocity_publisher = rospy.Publisher(self.topics["publishers"]["velocity"], Twist)
        self.humans = 0
        self.human_detected = False
        self.human_reached = False
        self.conf_dir = config
        if config[len(config) - 1] != '/':
            self.conf_dir += '/'
        self.personality = Personality()
        self.smartband_connected = False
        self.pose = {"x": 0.0, "y": 0.0, "z": 0.0}
        #TODO read it from file
        self.stop_distance = 0.8 #Minimum stop distance for kinectv1 sensor
        self.approach_linear_velocity = 0.5
        self.approach_angular_velocity = 0.3
        self.init_parameters()
        rospy.init_node("human_approach", anonymous=True)

    def init_parameters(self):
        if os.path.isfile(self.conf_dir + 'velocity.xml'):
            velocity_tree = ET.parse(self.conf_dir + 'velocity.xml')
            velocity_root = velocity_tree.getroot()
            if velocity_root.find('linear') is not None:
                self.approach_linear_velocity = float(velocity_root.find('linear').text)
            if velocity_root.find('angular') is not None:
                self.approach_angular_velocity = float(velocity_root.find('angular').text)
        else:
            print "Error : Can't find " + self.conf_dir + 'velocity.xml no such file or directory!'

        if not os.path.isfile(self.conf_dir + 'personality.xml'):
            print "Error : Can't find " + self.conf_dir + 'velocity.xml no such file or directory!'

        personality_tree = ET.parse(self.conf_dir + 'personality.xml')
        personality_root = personality_tree.getroot()

        if personality_root.find('extraversion') is not None:
            self.personality.extraversion = float(personality_root.find('extraversion').text)

        if personality_root.find('agreebleness') is not None:
            self.personality.agreebleness = float(personality_root.find('agreebleness').text)

        if personality_root.find('concientiouness') is not None:
            self.personality.concientiouness = float(personality_root.find('concientiouness').text)

        if personality_root.find('neuroticism') is not None:
            self.personality.neuroticism = float(personality_root.find('neuroticism').text)

        if personality_root.find('openness') is not None:
            self.personality.openness = float(personality_root.find('openness').text)

    def process_personality(self, personality_msg):
        self.personality.extraversion = personality_msg.extraversion
        self.personality.agreebleness = personality_msg.agree
        self.personality.concientiouness = personality_msg.concientiouness
        self.personality.neuroticism = personality_msg.neuroticism
        self.personality.openness = personality_msg.openness

    def process_velocity(self, velocity_msg):
        self.approach_linear_velocity = velocity_msg.linear.x
        self.approach_angular_velocity = velocity_msg.angular.z

    def process_odometry(self, odometry):
        if self.humans != len(odometry.markers) and len(odometry.markers) > 0:
            print "++ humans detected: " + str(self.humans) + " ++"

        self.humans = len(odometry.markers)
        if self.humans >= 1:
            self.human_detected = True
            self.pose["x"] = odometry.markers[0].pose.position.x
            self.pose["y"] = odometry.markers[0].pose.position.y
            self.pose["z"] = odometry.markers[0].pose.position.z
        else:
            self.human_detected = False
            self.pose["x"] = 0.0
            self.pose["y"] = 0.0
            self.pose["z"] = 0.0

    def process_stop_distance(self, sb_sensors):
        rospy.wait_for_service("compute_stop_distance")

        stop_distance_data = StopDistanceData()
        stop_distance_data.sbsensors = sb_sensors
        stop_distance_data.pdata = self.personality
        stop_distance_data.velocity = self.approach_linear_velocity

        try:
            compute_stop_distance = rospy.ServiceProxy('compute_stop_distance', ComputeStopDistance)
            new_stop_distance = compute_stop_distance(stop_distance_data)
            print 'process_stop_distance : '+str(self.stop_distance)+" -> "+str(new_stop_distance)
            self.stop_distance = new_stop_distance
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def approach(self):
        velocity = Twist()
        rate = rospy.Rate(10)
        #TODO delete this line it's only for test purposes
        self.smartband_connected = True
        print 'approach cycle started'
        while not rospy.is_shutdown():
            if self.smartband_connected and self.human_detected: #If human is tracked and smartband is connected
                #move toward human
                if self.pose["x"] > self.stop_distance:
                    velocity.linear.x = self.approach_linear_velocity
                    self.human_reached = False
                else:
                    print "**** Human detected: reached ****"
                    velocity.linear.x = 0.0
                    self.human_reached = False
                #turn toward human
                velocity.angular.z = min(1.0, self.pose["y"]) * self.approach_angular_velocity
            elif not self.human_detected:
                velocity.angular.z = self.approach_angular_velocity #turn around to look for a human
            self.velocity_publisher.publish(velocity)
            rate.sleep()


if __name__ == "__main__":
    try:
        config_dir = sys.argv[1]
        ha = HumanApproach(base_topics, config_dir)
        ha.approach()
    except rospy.ROSInterruptException:
        print "aproach interrupt exception"
