#!/usr/bin/env python

import rospy
import base

from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from lurch.msg import Control

class Tracking(base.Base):
    def __init__(self, nameSpace):
        super(Tracking, self).__init__()

        rospy.init_node("tracking")

        self.ns = nameSpace

        self.odoSub = rospy.Subscriber(self.ns["sub"]["odometry"], MarkerArray, self.processOdometry)
        self.velPub = rospy.Publisher(self.ns["pub"]["cmd_vel_avoid"], Twist, queue_size=10)
        self.conPub = rospy.Publisher(self.ns["pub"]["control"], Control, queue_size=10)

        self.pose = {"x":0.0, "y":0.0, "z":0.0}
        self.humans = 0

    def caFlag(self, flag):
        conMsgNew = Control()
        conMsgNew.collision_avoidance = 1
        conMsgNew.wandering = 1 - flag
        conMsgNew.tracking = flag
        return conMsgNew

    def processOdometry(self, odoMsg):
        # get first marker pose (we track the first human detected)
        self.humans = len(odoMsg.markers)
        if self.humans >= 1:
            self.pose["x"] = odoMsg.markers[0].pose.position.x
            self.pose["y"] = odoMsg.markers[0].pose.position.y
            self.pose["z"] = odoMsg.markers[0].pose.position.z
            self.conPub.publish(self.caFlag(1))
        else:
            self.pose["x"] = 0.0
            self.pose["y"] = 0.0
            self.pose["z"] = 0.0
            self.conPub.publish(self.caFlag(0))

    def tracking(self, refresh_r):
        vel  = Twist()
        rate = rospy.Rate(refresh_r)
        while not rospy.is_shutdown():
            if self.humans > 0:
                if self.pose["x"] >= 2.0:
                    rospy.loginfo("*** human detected: moving torwards ***")
                    vel.linear.x = 0.25
                else:
                    rospy.loginfo("human reached")
                    vel.linear.x = 0.0
                # turn smoothly torwards human
                vel.angular.z = min(1.0, self.pose["y"]) * 0.5
            #else:
            #    vel.linear.x = 0.0
            #    vel.angular.z = 0.0
                self.velPub.publish(vel)
            rate.sleep()

if (__name__ == "__main__"):
    Tracking(base.ns).tracking(10)
