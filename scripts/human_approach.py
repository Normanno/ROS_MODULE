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
from std_msgs.msg import Int32
from sgr_project.msg import StopDistance

from base_class import Base
from sensor_msgs.msg import PointCloud

#TODO ADD LOGGING FUNCTION


class HumanApproach(Base):

    def __init__(self, topics, config=''):
        super(HumanApproach, self).__init__()
        self.topics = topics

        self.conf_dir = config
        if config[len(config) - 1] != '/':
            self.conf_dir += '/'
        self.velocity_file_path = self.conf_dir + 'velocity.xml'
        self.stop_distance_file_path = self.conf_dir + 'stop_distance.xml'

        self.humans = 0
        self.human_detected = False
        self.human_reached = True
        self.smartband_connected = False
        self.rosaria_connected = self.ros_aria_connected
        self.pose = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.front_sonar = {"l": None, "r": None}
        self.front_obstacle = False
        self.stop_distance = 0.0
        self.stop_distance_velocity_delta = 0.0
        self.adaptation_enabled = False
        self.approach_linear_velocity = 0.0
        self.approach_angular_velocity = 0.0
        self.actual_velocity = Twist()

        self.uid = Int32()
        self.uid = -1  # for generic tests

        self.stop_detected_time = 0.0
        self.null_velocity_time = 0.0

        # To enable disable autonomous movements
        self.autonomous = False

        self.init_parameters()

        self.odometry_subscriber = rospy.Subscriber(self.topics["subscribers"]["odometry"],
                                                    MarkerArray, self.process_odometry, queue_size=1)
        self.sonar_subscriber = rospy.Subscriber(self.topics["subscribers"]["sonar"], PointCloud,
                                                 self.process_sonar, queue_size=3)
        self.velocity_ctrl_subscriber = rospy.Subscriber(topics["subscribers"]["velocity_ctrl"],
                                                         Twist, self.process_velocity, queue_size=10)
        self.adaptation_subscriber = rospy.Subscriber(topics['subscribers']['adaptation_ctrl'],
                                                      Bool, self.process_adaptation, queue_size=1)
        self.autonomous_subscriber = rospy.Subscriber(topics['subscribers']['autonomous_ctrl'],
                                                      Bool, self.process_autonomous, queue_size=1)
        self.restart_subscriber = rospy.Subscriber(topics['subscribers']['approach_ctrl'],
                                                   Bool, self.restart_approach, queue_size=1)
        self.velocity_publisher = rospy.Publisher(self.topics["publishers"]["velocity"],
                                                  Twist, queue_size=1)
        self.human_reached_publisher = rospy.Publisher(self.topics["publishers"]["human_reached"],
                                                       Bool, queue_size=10)
        self.smartband_state_subscriber = rospy.Subscriber(topics["subscribers"]["smartband_state"],
                                                           Bool, self.process_smartband_state, queue_size=1)
        self.stop_distance_subscriber = rospy.Subscriber(topics["subscribers"]["stop_distance"],
                                                         StopDistance, self.process_stop_distance, queue_size=1)

        self.user_info_subscriber = rospy.Subscriber(base_topics["subscribers"]["user_info_ctrl"],
                                                     Int32, self.process_user_info, queue_size=1)

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

    def restart_approach(self, restart_msg):
        #if bool(restart_msg.data) and self.human_reached:
        #    self.human_reached = False
        self.human_reached = False

    def process_user_info(self, user_info_msg):
        self.uid.data = user_info_msg.data

    def process_smartband_state(self, smartband_state_msg):
        if self.smartband_connected and not smartband_state_msg.data:
            print "** Smartband : disconnected **"
        elif not self.smartband_connected and smartband_state_msg.data:
            print "** Smartband : connected **"
        self.smartband_connected = smartband_state_msg.data

    def process_stop_distance(self, stop_distance_msg):
        if self.stop_distance != stop_distance_msg.distance or self.stop_distance_velocity_delta != stop_distance_msg.delta:
            print 'stop_distance ' + str(self.stop_distance) + '->' + str(stop_distance_msg.distance)
            print 'delta '+str(self.stop_distance_velocity_delta) + '->' + str(stop_distance_msg.delta)
        self.stop_distance = stop_distance_msg.distance
        self.stop_distance_velocity_delta = stop_distance_msg.delta

    def process_velocity(self, velocity_msg):
        self.approach_linear_velocity = velocity_msg.linear.x
        self.approach_angular_velocity = velocity_msg.angular.z
        #TODO test correctness of this exclusion
        if not self.human_reached:
            self.actual_velocity.linear.x = self.approach_linear_velocity
            self.velocity_publisher.publish(self.actual_velocity)

    def process_odometry(self, odometry_msg):
        if self.humans < len(odometry_msg.markers):
            print "++ humans detected: " + str(self.humans) + "->" + str(len(odometry_msg.markers)) + " ++"
        elif self.humans > len(odometry_msg.markers):
            print "-- humans detected: " + str(self.humans) + "->" + str(len(odometry_msg.markers)) + " --"

        self.humans = len(odometry_msg.markers)
        if self.humans >= 1:
            self.human_detected = True
            # OpenPtrack publish distance in feet, we use meters
            # Axis as in rep 103, x->forward y->left z->up
            self.pose["x"] = odometry_msg.markers[0].pose.position.x
            self.pose["y"] = odometry_msg.markers[0].pose.position.y
            self.pose["z"] = odometry_msg.markers[0].pose.position.z
        else:
            self.human_detected = False
            self.pose["x"] = 0.0
            self.pose["y"] = 0.0
            self.pose["z"] = 0.0

    def process_sonar(self, sonar_msg):
        self.front_sonar["l"] = sonar_msg.points[3].x
        self.front_sonar["r"] = sonar_msg.points[4].x

    def process_autonomous(self, autonomous_msg):
        self.autonomous = autonomous_msg.data

    def process_adaptation(self, adaptation_msg):
        self.adaptation_enabled = adaptation_msg.data

    def publish_human_reached(self):
        new_msg = Bool()
        new_msg.data = self.human_reached
        self.human_reached_publisher.publish(new_msg)

    def update_velocity(self, x_linear, z_angular):
        update = False

        if x_linear != self.actual_velocity.linear.x:
            self.actual_velocity.linear.x = x_linear
            update = True

        if z_angular != self.actual_velocity.angular.z:
            self.actual_velocity.angular.z = z_angular
            update = True

        return update

    def print_state(self):
        print "**********Actual State***********"
        print "-RosAria : " + ("Connected " if self.ros_aria_connected else "Disconnected")
        print "-Smartband : " + ("Connected " if self.smartband_connected else "Disconnected")
        print "-Human Detected : " + ("Yes" if self.human_detected else "No")
        print "-Human Reached : " + ("Yes" if self.human_reached else "No")
        print "-Autonomous Movement : " + ("Enabled" if self.autonomous else "Disabled")
        print "-Front Obstacle : " + ("Yes" if self.front_obstacle else "No")
        print "-Stop Distance : " + str(self.stop_distance)
        print "--Delta : " + str(self.stop_distance_velocity_delta)
        print "-Actual Linear Velocity : " + str(self.actual_velocity.linear.x)
        print "-Actual Angular Velocity : " + str(self.actual_velocity.angular.z)
        print "********End Actual State*********"

    def near_human(self):
        # if the detected pose is range or the sonar detect an obstacle
        return self.pose["x"] <= (self.stop_distance + self.stop_distance_velocity_delta)

    def detect_front_obstacle(self):
        evaluate = lambda x: x < self.stop_distance + self.stop_distance_velocity_delta
        if self.front_sonar["l"] is not None and self.front_sonar["r"] is not None:
            self.front_obstacle = evaluate(self.front_sonar["l"]) or evaluate(self.front_sonar["r"])
            self.front_sonar["l"] = None
            self.front_sonar["r"] = None

    def autonomous_movement(self):
        #print 'autonomous_movement ' + str(self.autonomous ) +' - human reached ' + str(self.human_reached)
        return self.autonomous or (not self.autonomous and not self.human_reached)

    def can_move(self):
        return self.autonomous_movement() and self.smartband_connected and self.human_detected

    def look_around(self):
        return self.autonomous_movement() and self.smartband_connected and not self.human_detected

    def stop_there(self):
        return self.autonomous_movement() and not self.smartband_connected

    def approach(self):
        frequency = 60
        rate = rospy.Rate(frequency)
        velocity_update = False
        counter = 0

        self.print_state()
        start_at_distance = 0.0
        stop_at_distance = 0.0
        start_at_time = 0.0
        stop_at_time = 0.0

        while not rospy.is_shutdown():
            self.detect_front_obstacle()
            if self.can_move():
                if counter == 10:
                    counter = 0
                counter += 1
                # ROSAria mantain the velocity
                # move toward human
               # print 'front-obstacle '+ str(self.front_obstacle)
               # print 'autonomous_movement ' + str(self.autonomous) + ' - human reached ' + str(self.human_reached)
                print ' autonomous_movement_function ' + str(self.autonomous_movement())
                print ' near-human ' + str(self.near_human())
                self.print_state()
                if not self.near_human() and not self.front_obstacle:
                    if self.human_reached:
                        print "**** Human detected: MOVING ****"
                        self.human_reached = False
                        velocity_update = self.update_velocity(self.approach_linear_velocity, self.actual_velocity.angular.z)
                        start_at_distance = self.pose["x"]
                        # %f = microsedonds
                        start_at_time = datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S.%f')
                elif self.near_human() or self.front_obstacle:
                    if not self.human_reached:
                        print "**** Human detected: REACHED ****"
                        stop_at_distance = self.pose["x"]
                        stop_at_time = datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S.%f')
                        stop_distance_log_msg = "x " + str(self.pose["x"]) + " - sd " + str(self.stop_distance) + " - " + str(self.stop_distance_velocity_delta)
                        rospy.loginfo("TIME [" + str() + "]" + stop_distance_log_msg)
                        self.human_reached = True
                        velocity_update = self.update_velocity(0.0, self.actual_velocity.angular.z)
                        if self.log_active:
                            self.logger.log_experiment(self.uid, start_at_time, stop_at_time, start_at_distance,
                                                       stop_at_distance, self.stop_distance,
                                                       self.stop_distance_velocity_delta)
                # turn toward human
                new_angular_velocity = min(1.0, self.pose["y"]) * self.approach_angular_velocity
                velocity_update = velocity_update or self.update_velocity(self.actual_velocity.linear.x, new_angular_velocity)
            elif self.look_around() and not self.front_obstacle:
                # turn around looking for a human
                velocity_update = self.update_velocity(0.0, self.approach_angular_velocity)
            # if smartband is not connected
            elif self.stop_there() or self.front_obstacle:
                velocity_update = self.update_velocity(0.0, 0.0)

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
        print "Aproach interrupt exception"
