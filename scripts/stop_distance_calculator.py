#!/usr/bin/env python

import rospy
import StringIO
import os
import sys
from base_class import Base
from xml.etree import ElementTree as ET
from sgr_project.msg import ApproachData
from sgr_project.msg import SmartbandSensors
from sgr_project.msg import Personality
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from base_class import topics as base_topics
import matlab.engine


engine = matlab.engine.start_matlab()
out = StringIO.StringIO()
err = StringIO.StringIO()
ret = engine.dec2hex(2 ** 60, stdout=out, stderr=err)


class StopDistanceCalculator(Base):

    def __init__(self, topics, config=''):
        super(StopDistanceCalculator, self).__init__()
        self.conf_dir = config

        if config[len(config) - 1] != '/':
            self.conf_dir += '/'

        self.personality_file_path = self.conf_dir + 'personality.xml'
        self.stop_distance_file_path = self.conf_dir + 'stop_distance.xml'

        self.new_smartband_data = False
        self.smartband_sensors = SmartbandSensors()
        self.new_personality_data = False
        self.personality = Personality()
        self.new_velocity_data = False
        self.velocity = 0.0
        self.human_reached = False

        self.minimum_stop_distance = 0.0
        self.stop_distance = 0.0
        self.smartband_detected = False
        #TODO check if vale change on rosaria connection
        self.motors_enabled = self.ros_aria_connected

        self.smartband_subscriber = rospy.Subscriber(topics["subscribers"]["smartband"],
                                                     SmartbandSensors, self.process_smartband)
        self.motors_subscriber = rospy.Subscriber(topics["subscribers"]["motors"],
                                                  Bool, self.process_motors)
        self.personality_subscriber = rospy.Subscriber(topics["subscribers"]["personality_ctrl"],
                                                       Personality, self.process_personality)
        self.human_reached_subscriber = rospy.Subscriber(topics["subscribers"]["human_reached"],
                                                         Bool, self.process_human_reached, queue_size=10)
        self.velocity_subsciber = rospy.Subscriber(topics["subscribers"]["velocity"],
                                                   Twist, self.process_velocity, queue_size=10)
        self.approach_data_publisher = rospy.Publisher(topics["publishers"]["approach"],
                                                       ApproachData, queue_size=10)
        self.init_parameters()
        rospy.init_node("stop_distance_calculator", anonymous=True)

    def init_parameters(self):
        if os.path.isfile(self.personality_file_path):
            personality_tree = ET.parse(self.personality_file_path)
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
        else:
            print "Error : Can't find " + self.personality_file_path + 'no such file or directory!'

        if os.path.isfile(self.stop_distance_file_path):
            stop_distance_tree = ET.parse(self.stop_distance_file_path)
            stop_distance_root = stop_distance_tree.getroot()
            if stop_distance_root.find('initial') is not None:
                self.stop_distance = float(stop_distance_root.find('initial').text)
            if stop_distance_root.find('minimum') is not None:
                self.minimum_stop_distance = float(stop_distance_root.find('minimum').text)
        else:
            print "Error : Can't find " + self.stop_distance_file_path + 'no such file or directory!'

    def process_motors(self, motors_message):
        self.motors_enabled = motors_message.data

    def process_human_reached(self, human_reached_message):
        self.human_reached = human_reached_message.data

    def process_smartband(self, smartband_msg):
        self.smartband_sensors = smartband_msg
        self.new_smartband_data = True

    def process_velocity(self, velocity_msg):
        #Takes linear velocity
        if self.velocity != velocity_msg.linear.x:
            self.velocity = velocity_msg.linear.x
            self.new_velocity_data = True

    def process_personality(self, personality_msg):
        self.personality = personality_msg
        self.new_personality_data = True

    def publish_approach_msg(self):
        approach_msg = ApproachData()
        approach_msg.stop_distance = self.stop_distance
        approach_msg.smartband_detected = self.smartband_detected
        self.approach_data_publisher.publish(approach_msg)

    def smartband_sensors_to_values(self):
        values = list()
        values.append(self.smartband_sensors.accx)
        values.append(self.smartband_sensors.accy)
        values.append(self.smartband_sensors.accz)
        values.append(self.smartband_sensors.gyrox)
        values.append(self.smartband_sensors.gyroy)
        values.append(self.smartband_sensors.gyroz)
        values.append(self.smartband_sensors.hr)
        values.append(self.velocity)
        values.append(self.personality.extraversion)
        values.append(self.personality.agreebleness)
        values.append(self.personality.concientiouness)
        values.append(self.personality.neuroticism)
        values.append(self.personality.openness)
        return values

    def calc_cycle(self):
        rate = 10
        ros_rate = rospy.Rate(10)
        counter = 0
        # first start message
        self.publish_approach_msg()
        while not rospy.is_shutdown():
            # if the human is not at the computed stop distance and the motors are enabled
            # then calculate the stop distance
            print 'motors_enabled '+str(self.motors_enabled)
            if not self.human_reached and self.motors_enabled:
                # if there is some new data and is not the first start
                if self.new_personality_data or self.new_smartband_data or self.new_velocity_data:
                    self.smartband_detected = self.smartband_detected or self.new_smartband_data
                    self.new_velocity_data = False
                    self.new_personality_data = False
                    self.new_smartband_data = False
                    values = self.smartband_sensors_to_values()
                    #Calculate stop distance with matlab engine
                    new_stop_distance = engine.getStopDistanceSGR(values, nargout=1)
                    #Due to kinect v1 limitations, there is a minimum stop distance
                    if new_stop_distance >= self.minimum_stop_distance:
                        self.stop_distance = new_stop_distance
                    else:
                        self.stop_distance = self.minimum_stop_distance
                    counter = 0
                    self.publish_approach_msg()
                # if there is no new data
                else:
                    counter += 1
                    # if there is no data for one second, it is assumed that
                    # the smartband is disconnected
                    if counter >= rate:
                        self.smartband_detected = False
                        self.publish_approach_msg()
                        counter = rate

            ros_rate.sleep()


if __name__ == '__main__':
    print 'starting calculator node'
    config_dir = sys.argv[1]
    try:
        calc_node = StopDistanceCalculator(base_topics, config_dir)
        calc_node.calc_cycle()
    except rospy.ROSInterruptException:
        print "aproach interrupt exception"