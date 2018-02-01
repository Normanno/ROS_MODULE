#!/usr/bin/env python

import rospy
import sys
import os
import time
from xml.etree import ElementTree as ET
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import Int32
from base_class import Base
from base_class import topics
from sgr_project.msg import Personality


class ParametersPublisher(Base):

    options = {
        "1": "Update personality parameters",
        "2": "Display actual personality",
        "3": "Update linear velocity",
        "4": "Display linear velocity",
        "5": "Update angular velocity",
        "6": "Display angular velocity",
        "7": "Enable/Disable continuous movements",
        "r": "Restart approaching behaviour (useful in case of \n\t\tcontinuous movement disabled)",
        "b": "Robot go back! (0.2 m/s for 1 second",
        "s": "Stop the robot (set angular and linear velocity to 0)",
        "h": "Display all options",
        "e": "exit"
    }

    def __init__(self, base_topics, conf):
        super(ParametersPublisher, self).__init__()
        self.personality_publisher = rospy.Publisher(base_topics["publishers"]["personality_ctrl"], Personality)
        self.velocity_publisher = rospy.Publisher(base_topics["publishers"]["velocity_ctrl"], Twist)
        #TODO add enable disable adaptations publisher
        self.adaptation_publisher = rospy.Publisher(base_topics["publishers"]["adaptation_ctrl"], Bool)
        self.autonomous_publisher = rospy.Publisher(base_topics["publishers"]["autonomous_ctrl"], Bool)
        self.approach_publisher = rospy.Publisher(base_topics["publishers"]["approach_ctrl"], Bool)
        self.user_info_publisher = rospy.Publisher(base_topics["publishers"]["user_info_ctrl"], Int32)
        self.options_functions = {
                "1": self.update_personality,
                "2": self.display_actual_personlity,
                "3": self.update_linear_velocity,
                "4": self.display_actual_linear_velocity,
                "5": self.update_angular_velocity,
                "6": self.display_actual_angular_velocity,
                "7": self.control_continuous_movement,
                "8": self.control_adaptation,
                "r": self.restart_approach,
                "b": self.go_back,
                "s": self.stop_robot,
                "h": self.display_options
            }
        self.conf_dir = conf
        if conf[len(conf)-1] != '/':
            self.conf_dir += '/'

        self.velocity = Twist()
        self.personality = Personality()
        self.uid = Int32()
        self.uid.data = -1
        self.init_parameters()
        rospy.init_node('parameters_publisher', anonymous=True)

    def init_parameters(self):
        if os.path.isfile(self.conf_dir + 'velocity.xml'):
            velocity_tree = ET.parse(self.conf_dir + 'velocity.xml')
            velocity_root = velocity_tree.getroot()
            if velocity_root.find('linear') is not None:
                self.velocity.linear.x = float(velocity_root.find('linear').text)
            if velocity_root.find('angular') is not None:
                self.velocity.angular.z = float(velocity_root.find('angular').text)
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

    def keyboard_input_loop(self):
        exit_loop = False
        choice = None
        self.display_options()
        while not exit_loop:
            choice = raw_input("Option: ")
            choice = str(choice)
            if choice in ParametersPublisher.options.keys() and choice != "e":
                self.options_functions[choice]()
            elif choice == "e":
                exit_loop = True
            else:
                print "Wrong input"

    def restart_approach(self):
        msg = Bool()
        msg.data = True
        self.approach_publisher.publish()

    def publish_velocity(self):
        self.velocity_publisher.publish(self.velocity)

    def update_linear_velocity(self):
        new_linear_velocity = input("Linear velocity (actual "+str(self.velocity.linear.x)+") :")
        try:
            new_linear_velocity_value = float(new_linear_velocity)
            self.velocity.linear.x = new_linear_velocity_value
        except ValueError:
            print "ERROR:" + str(new_linear_velocity) + " is not a number!"
            return
        self.publish_velocity()

    def display_actual_linear_velocity(self):
        print "Actual linear velocity : " + str(self.velocity.linear.x)

    def update_angular_velocity(self):
        new_angular_velocity = input("Angular velocity (actual "+str(self.velocity.angular.z)+") :")
        try:
            new_angular_velocity_value = float(new_angular_velocity)
            self.velocity.angular.z = new_angular_velocity_value
        except ValueError:
            print "ERROR:" + str(new_angular_velocity) + " is not a number!"
            return
        self.publish_velocity()

    def display_actual_angular_velocity(self):
        print "Actual angular velocity : " + str(self.velocity.angular.z)

    def control_continuous_movement(self):
        string_prompt = "Choices:\n(1)Enable continuous movement\n(2)Disable continuous movement\nChoice: "
        choice = -1
        msg = Bool()

        try:
            choice = int(raw_input(string_prompt))
        except ValueError:
            choice = 2

        if choice == 1:
            print 'enabling autonomous movement'
            msg.data = True
            self.autonomous_publisher.publish(msg)
        elif choice == 2:
            print 'disabling autonomous movement'
            msg.data = False
            self.autonomous_publisher.publish(msg)
        else:
            print "Wrong choice"

    def control_adaptation(self):
        string_prompt = "Choices:\n(1)Enable adaptation\n(2)Disable adaptation\nChoice: "
        choice = -1
        msg = Bool()

        try:
            choice = int(raw_input(string_prompt))
        except ValueError:
            choice = 2

        if choice == 1:
            print 'enabling adaptation'
            msg.data = True
            self.adaptation_publisher.publish(msg)
        elif choice == 2:
            print 'disabling adaptation'
            msg.data = False
            self.adaptation_publisher.publish(msg)
        else:
            print "Wrong choice"

    def go_back(self):
        self.velocity.linear.x = -0.2
        self.publish_velocity()
        time.sleep(1)
        self.stop_robot()

    def stop_robot(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.publish_velocity()

    def display_options(self):
        print "Available options: "
        keys = ParametersPublisher.options.keys()
        keys.sort()
        for k in keys:
            print "\t-" + k + " : " + ParametersPublisher.options.get(k)

    def extract_data_from_csv(self, csv_path):
        extraversion_v = 0.0
        agreebleness_v = 0.0
        concientiouness_v = 0.0
        neuroticism_v = 0.0
        openness_v = 0.0
        counter = 0
        csv_fields = list()
        answers = list()
        get_ans = lambda i: int(str(answers[i-1]).replace("\"", ""))
        get_ans_r = lambda i: 6 - get_ans(i)
        with open(csv_path, 'r') as csv_file:
            for line in csv_file:
                csv_fields = str(line).replace('\n', '').strip().split(',')
                #if counter != 0:
                #   csv_fields = str(line).replace('\n', '').strip().split(',')
                #counter += 1

        #the first field is date&time the second is uid
        answers = csv_fields[2:]
        self.uid.data = int(str(csv_fields[1]).replace("\"", ""))
        extraversion_v = get_ans(1) + get_ans_r(6) + get_ans(11) + get_ans(16) + get_ans_r(21) + get_ans(26) + get_ans_r(31) + get_ans(36)
        agreebleness_v = get_ans_r(2) + get_ans(7) + get_ans_r(12) + get_ans(17) + get_ans(22) + get_ans_r(27) + get_ans(32) + get_ans_r(37) + get_ans(42)
        concientiouness_v = get_ans(3) + get_ans_r(8) + get_ans(13) + get_ans_r(18) + get_ans_r(23) + get_ans(28) + get_ans(33) + get_ans(38) + get_ans_r(43)
        neuroticism_v = get_ans(4) + get_ans_r(9) + get_ans(13) + get_ans(19) + get_ans_r(24) + get_ans(29) + get_ans(34) + get_ans(39)
        openness_v = get_ans(5) + get_ans(10) + get_ans(15) + get_ans(20) + get_ans(25) + get_ans(30) + get_ans_r(35) + get_ans(40) + get_ans_r(41) + get_ans(44)

        return extraversion_v, agreebleness_v, concientiouness_v, neuroticism_v, openness_v

    def update_personality(self):
        print "***Updating personality parameters***"
        check_value = lambda x, y: x if y == '' else float(y)
        u_choice = raw_input("Update (1 default):\n\t1)Insert manually\n\t2)Insert and calculate from csv\nChoice: ")
        u_choice = int(u_choice)
        print 'Choice : ' + str(u_choice)
        if u_choice != '' and u_choice == 2:
            print 'csv'
            csv_path = raw_input("Insert id.csv path :")
            extraversion_v, agreebleness_v, concientiouness_v, neuroticism_v, openness_v = self.extract_data_from_csv(csv_path)
        else:
            try:
                extraversion_v = check_value(self.personality.extraversion,
                                                 raw_input("Extraversion (actual " + str(self.personality.extraversion) + ") : "))
                agreebleness_v = check_value(self.personality.agreebleness,
                                                 raw_input("Agreebleness (actual " + str(self.personality.agreebleness) + ") : "))
                concientiouness_v = check_value(self.personality.concientiouness,
                                                    raw_input("Concientioness (actual " + str(self.personality.concientiouness) + ") : "))
                neuroticism_v = check_value(self.personality.neuroticism,
                                                raw_input("Neuroticism (actual " + str(self.personality.neuroticism) + ") : "))
                openness_v = check_value(self.personality.openness,
                                             raw_input("Openness (actual " + str(self.personality.openness) + ") : "))
                self.uid.data = -1
            except ValueError:
                print "ERROR: not a number!"
                return
        self.personality.uid = self.uid.data
        self.personality.extraversion = extraversion_v
        self.personality.agreebleness = agreebleness_v
        self.personality.concientiouness = concientiouness_v
        self.personality.neuroticism = neuroticism_v
        self.personality.openness = openness_v
        self.personality_publisher.publish(self.personality)
        #self.user_info_publisher.publish(self.uid)
        print "***Parameters successfully updated***"

    def display_actual_personlity(self):
        print "***Actual personality parameters***"
        print "\t-Extraversion : " + str(self.personality.extraversion)
        print "\t-Agreebleness : " + str(self.personality.agreebleness)
        print "\t-Concientioness : " + str(self.personality.concientiouness)
        print "\t-Neuroticism : " + str(self.personality.neuroticism)
        print "\t-Openness : " + str(self.personality.openness)
        print "***********************************"


if __name__ == '__main__':
    print "Wellcome"
    config_dir = sys.argv[1]
    personality_publisher = ParametersPublisher(topics, config_dir)
    personality_publisher.keyboard_input_loop()
    print "Good Bye!"