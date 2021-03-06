#!/usr/bin/env python

import rospy
import StringIO
import os
import sys
import time
from base_class import Base
from xml.etree import ElementTree as ET
from sgr_project.msg import SmartbandSensors
from sgr_project.msg import Personality
from sgr_project.msg import StopDistance
from std_msgs.msg import Bool
from std_msgs.msg import Int32
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
        self.smartband_data_last_timestamp = 0
        self.new_personality_data = False
        self.personality = Personality()
        self.new_velocity_data = False
        self.velocity = 0.0
        self.human_reached = False
        self.adaptation_enabled = False

        self.minimum_stop_distance = 0.0
        self.stop_distance = 0.0
        self.smartband_detected = False
        self.motors_enabled = self.ros_aria_connected

        self.uid = Int32()
        self.uid.data = -1

        self.adaptation_subscriber = rospy.Subscriber(topics['subscribers']['adaptation_ctrl'],
                                                      Bool, self.process_adaptation, queue_size=1)

        self.smartband_subscriber = rospy.Subscriber(topics["subscribers"]["smartband"],
                                                     SmartbandSensors, self.process_smartband,
                                                     queue_size=5)
        self.motors_subscriber = rospy.Subscriber(topics["subscribers"]["motors"],
                                                  Bool, self.process_motors, queue_size=1)
        self.personality_subscriber = rospy.Subscriber(topics["subscribers"]["personality_ctrl"],
                                                       Personality, self.process_personality)
        self.human_reached_subscriber = rospy.Subscriber(topics["subscribers"]["human_reached"],
                                                         Bool, self.process_human_reached, queue_size=1)
        self.velocity_subscriber = rospy.Subscriber(topics["subscribers"]["velocity"],
                                                    Twist, self.process_velocity, queue_size=1)
        #TODO verify double velocity reads
        self.velocity_ctrl_subscriber = rospy.Subscriber(topics["subscribers"]["velocity_ctrl"],
                                                         Twist, self.process_velocity, queue_size=1)

        self.smartband_state_publisher = rospy.Publisher(topics["publishers"]["smartband_state"],
                                                         Bool, queue_size=1)
        self.stop_distance_publisher = rospy.Publisher(topics["publishers"]["stop_distance"],
                                                       StopDistance, queue_size=1)
        self.user_info_subscriber = rospy.Subscriber(base_topics["subscribers"]["user_info_ctrl"],
                                                     Int32, self.process_user_info, queue_size=1)
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
            if personality_root.find('concientiounsness') is not None:
                self.personality.concientiouness = float(personality_root.find('concientiounsness').text)
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
            if stop_distance_root.find('adaptation') is not None:
                self.adaptation_enabled = bool(stop_distance_root.find('adaptation').text)
        else:
            print "Error : Can't find " + self.stop_distance_file_path + 'no such file or directory!'

    def process_user_info(self, user_info_msg):
        self.uid.data = user_info_msg.data

    def process_adaptation(self, adaptation_message):
        self.adaptation_enabled = adaptation_message.data

    def process_motors(self, motors_message):
        self.motors_enabled = motors_message.data

    def process_human_reached(self, human_reached_message):
        self.human_reached = human_reached_message.data

    def process_smartband(self, smartband_msg):
        self.smartband_sensors = smartband_msg
        self.new_smartband_data = True
        self.smartband_detected = True
        self.smartband_data_last_timestamp = time.time()

    def process_velocity(self, velocity_msg):
        #Takes linear velocity
        if self.velocity != velocity_msg.linear.x:
            self.velocity = velocity_msg.linear.x
            self.new_velocity_data = True

    def process_personality(self, personality_msg):
        self.personality = personality_msg
        self.uid.data = personality_msg.uid
        if self.log_active:
           self.init_user_log(self.uid.data, personality_msg)

        self.new_personality_data = True


    def evaluate_delta(self, velocity, deceleration=0.5):
        # distance traveled by the robot in 0.1 seconds (RosAria update frequency)
        distance = velocity * 0.15
        # Stop distance adaptation sum of :
        # - uniform decelleration distance to stop = v^2 / 2*decelleration
        # - robot walkable distance for late frame
        delta = 0.0
        if deceleration > 0.0:
            delta += pow(velocity, 2) / (2 * deceleration)

        #delta += 2 * distance
        return delta

    def smartband_sensors_to_values(self):
        values = list()
        values.append(float(self.smartband_sensors.accx))
        values.append(float(self.smartband_sensors.accy))
        values.append(float(self.smartband_sensors.accz))
        values.append(float(self.smartband_sensors.gyrox))
        values.append(float(self.smartband_sensors.gyroy))
        values.append(float(self.smartband_sensors.gyroz))
        values.append(float(self.smartband_sensors.hr))
        values.append(float(self.velocity))
        values.append(float(self.personality.extraversion))
        values.append(float(self.personality.agreebleness))
        values.append(float(self.personality.concientiouness))
        values.append(float(self.personality.neuroticism))
        values.append(float(self.personality.openness))
        return values

    def smartband_state_publish(self):
        msg = Bool()
        msg.data = self.smartband_detected
        self.smartband_state_publisher.publish(msg)

    def matlab_calculate_and_publish(self, values):
        new_stop_distance = engine.getStopDistanceSGRSVM(values, nargout=1)

        if new_stop_distance >= self.minimum_stop_distance:
            self.stop_distance = new_stop_distance
        else:
            self.stop_distance = self.minimum_stop_distance

        msg = StopDistance()
        msg.distance = self.stop_distance
        msg.delta = 0.0
        print str(values)
        print new_stop_distance
        if self.adaptation_enabled:
            #the velocity is in values at the 7-th
            #velocity = values[7]
            velocity = values[7]
            ##TODO take this value with dynamic_reconfigure
            deceleration = 0.5
            msg.delta = self.evaluate_delta(velocity, deceleration)

        if self.log_active:
            self.logger.log_smartband_data(self.uid.data, 'timestamp', values[:7], values[7],
                                           msg.distance, msg.delta, deceleration, self.human_reached)

        self.stop_distance_publisher.publish(msg)

    def check_new_data(self):
        return self.new_personality_data or self.new_smartband_data or self.new_velocity_data

    def init_user_log(self, uid, personality_msg):
        personality_dict = dict()
        personality_dict["extraversion"] = personality_msg.extraversion
        personality_dict["agreebleness"] = personality_msg.agreebleness
        personality_dict["concientiouness"] = personality_msg.concientiouness
        personality_dict["neuroticism"] = personality_msg.neuroticism
        personality_dict["openness"] = personality_msg.openness
        self.logger.init_user_logs(uid, personality_dict)

    def calc_cycle(self):
        rate = 10
        ros_rate = rospy.Rate(10)
        counter = 0
        last_smartband_check = 0
        print "starting calculator cycle"
        if not self.smartband_detected:
            print "** Smartband : disconnected **"

        if self.uid.data == -1:
            self.init_user_log(self.uid.data, self.personality)

        while not rospy.is_shutdown():
            # if there is no data for one second, it is assumed that
            # the smartband is disconnected
            if counter >= 2*rate :
                if self.smartband_detected and not self.new_smartband_data:
                    print "** Smartband : disconnected **"
                    self.smartband_detected = False

            self.smartband_state_publish()
            # if the human is not at the computed stop distance and the motors are enabled
            # then calculate the stop distance
            if not self.human_reached and self.check_new_data():
                # if there is some new data and is not the first start
                if counter > 0 and self.smartband_detected:
                    print "** Smartband : connected **"
                    counter = 0

                self.new_velocity_data = False
                self.new_personality_data = False
                self.new_smartband_data = False
                values = self.smartband_sensors_to_values()
                self.matlab_calculate_and_publish(values)

            elif self.motors_enabled and last_smartband_check == self.smartband_data_last_timestamp:
                counter += 1

            last_smartband_check = self.smartband_data_last_timestamp

            ros_rate.sleep()


if __name__ == '__main__':
    print 'starting calculator node'
    config_dir = sys.argv[1]
    try:
        calc_node = StopDistanceCalculator(base_topics, config_dir)
        calc_node.calc_cycle()
    except rospy.ROSInterruptException:
        print "aproach interrupt exception"
