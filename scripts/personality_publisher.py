#!/usr/bin/env python

import rospy
from base_class import Base
from base_class import topics
from sgr_project.msg import Personality


class PersonalityPublisher(Base):

    options = {
        "u": "Update personality parameters",
        "d": "Display actual parameters",
        "e": "exit"
    }


    def __init__(self, base_topics):
        super(PersonalityPublisher, self).__init__()
        self.personality_publisher = rospy.Publisher(base_topics["publishers"]["personality"], Personality)
        self.options_functions = {
                "u": self.update_personality
            }

    def keyboard_input_loop(self):
        exit_loop = False
        choice = None
        while not exit_loop:
            self.display_options()
            choice = input("Option: ")
            if choice in PersonalityPublisher.options.keys() and choice != "e":
                self.options_functions[choice]
            elif choice == "e":
                exit_loop = True
            else:
                print "Wrong input"


    def display_options(self):
        print "Available options: "
        for k in PersonalityPublisher.options.keys():
            print "\t-" + k + " : " + PersonalityPublisher.options.get(k)

    def update_personality(self):
        print "updating personality"

    def display_actual_personlity(self):
        print "displiing actual personality"


if __name__ == '__main__':
    print "Wellcome"
    personality_publisher = PersonalityPublisher(topics)
    personality_publisher.keyboard_input_loop()
    print "Good Bye!"