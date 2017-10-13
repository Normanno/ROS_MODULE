#!/usr/bin/env python

"""
    Based on nav_test.py - Version 1.1 2013-12-20

    Command a robot to move autonomously to a goal location defined in the map frame.
    Keep track of success rate, time elapsed, and total distance traveled.

    Originally created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import actionlib

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from lurch.msg import Control
from base import ns

class GotoLocation():
    def __init__(self, nameSpace):
        rospy.init_node('goto_location')

        self.ns = nameSpace

        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                           'SUCCEEDED', 'ABORTED', 'REJECTED',
                           'PREEMPTING', 'RECALLING', 'RECALLED',
                           'LOST']

        # Subscribe to goal topic
        self.movSub = rospy.Subscriber(self.ns["sub"]["move_base_goal"], Control, self.processLocation)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient(self.ns["sub"]["move_base"], MoveBaseAction)

        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher(self.ns["pub"]["cmd_vel"], Twist, queue_size=5)


        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")

        # A variable to hold the initial pose of the robot to be set by
        # the user in RViz
        self.initial_pose = PoseWithCovarianceStamped()

        self.location = ""
        self.last_location = ""

        # Get the initial pose from the user
        rospy.loginfo("*** Waiting for 2D initial pose estimate ***")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("goto_location node initialized")

        while not rospy.is_shutdown():
            if self.location != self.last_location:
                # Set up the next goal location
                goal = MoveBaseGoal()
                goal.target_pose.pose = self.location
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()

                # Let the user know where the robot is going next
                rospy.loginfo("Going to: " + str(self.location))

                # Start the robot toward the next location
                self.move_base.send_goal(self.goal)

                # Allow 5 minutes to get there
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

                # Check for success or failure
                if not finished_within_time:
                    self.move_base.cancel_goal()
                    rospy.loginfo("Timed out achieving goal")
                else:
                    state = self.move_base.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Goal succeeded!")
                        rospy.loginfo("State:" + str(state))
                    else:
                      rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))

                self.last_location = self.location

    def processLocation(self, data):
        self.location = data.pose

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        GotoLocation(ns)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL goto location finished.")
