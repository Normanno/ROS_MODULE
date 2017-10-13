#!/usr/bin/env python

import random
import copy
import rospy
import geometry_msgs.msg
import sensor_msgs.msg

from base import Base
from lurch.msg import Control

class CollisionAvoidance(Base):
    def __init__(self, nameSpace):
        super(CollisionAvoidance, self).__init__()

        rospy.init_node('collision_avoidance')

        # using a custom name space (dict)
        self.ns = nameSpace

        # subscribers
        self.sonSub = rospy.Subscriber(self.ns["sub"]["sonar"], sensor_msgs.msg.PointCloud, self.processSonar)
        self.velSub = rospy.Subscriber(self.ns["sub"]["cmd_vel_avoid"], geometry_msgs.msg.Twist, self.processCA)
        self.conSub = rospy.Subscriber(self.ns["sub"]["control"], Control, self.processControl)

        # publishers
        self.velPub = rospy.Publisher(self.ns["pub"]["cmd_vel"], geometry_msgs.msg.Twist, queue_size=10)

        # data holders
        self.velMsg = geometry_msgs.msg.Twist()
        self.angularZ = [-0.3, -0.3, -0.3, -0.3, -0.3]
        self.angularZNeg = [0.3, 0.3, 0.3, 0.3, 0.3]
        self.avoiding = False

    def processCA(self, velMsg):
        self.velMsg = velMsg

    def processControl(self, data):
        self.conMsg = data

    def collision_avoidance(self, refresh_r):
        vel = geometry_msgs.msg.Twist()
        rate = rospy.Rate(refresh_r)
        while not rospy.is_shutdown():
            if max(self.sonData[0:16]) > 0:
                # check frontal sonars to recognize obstacles
		if (self.sonData[1].x < 0.5 or self.sonData[2].x < 0.7 or self.sonData[4].x < 0.9 or self.sonData[3].x < 0.9 or self.sonData[5].x < 0.7 or self.sonData[6].x < 0.5):
		    #stop linear
                    vel.linear.x =  0.0
                    # can go front
                    if (self.sonData[1].x > 0.45 and self.sonData[2].x > 0.65 and self.sonData[3].x > 0.85 and
                       self.sonData[4].x > 0.85 and self.sonData[5].x > 0.65 and self.sonData[6].x > 0.45):
                        rospy.loginfo("go front")
                        vel.linear.x =  0.15
                        vel.angular.z = 0.0
                        self.avoiding = False
                    # check if can turn in one side
                    elif (self.sonData[7].y <= -0.25 and self.sonData[6].x >= 0.25 and self.sonData[5].x >= 0.25 and
                     self.sonData[0].y >= 0.25 and self.sonData[1].x >= 0.25 and self.sonData[2].x >= 0.25):
			if(not self.avoiding):
                            self.avoiding = True
                            if ((self.sonData[7].y * -2  + self.sonData[6].x * 0.8 + self.sonData[5].x * 0.5 + self.sonData[4].x * 0.2) >
			       (self.sonData[0].y * 2 + self.sonData[1].x * 0.8 + self.sonData[2].x * 0.5 + self.sonData[3].x * 0.2 )):
		                rospy.loginfo("-->")
			        vel.angular.z = random.choice(self.angularZ)
		 	    else:
			        rospy.loginfo("<--")
			        vel.angular.z = random.choice(self.angularZNeg)
			#self.conPub.publish(self.caFlag(1))
			#rospy.sleep(random.choice(self.sleepIntervals))
                    # if every side is too much obstacled, do a reverse
                    elif (self.sonData[9].x < -0.2 and self.sonData[10].x < -0.3 and self.sonData[11].x < -0.6 and
                        self.sonData[12].x < -0.6 and self.sonData[13].x < -0.3 and self.sonData[14].x < -0.2):
			self.avoiding = False
			rospy.loginfo("reverse")
                        vel.linear.x = -0.1
                        vel.angular.z = 0.0
                        #self.conPub.publish(self.caFlag(1))
                    self.velPub.publish(vel)
                else:
                    rospy.loginfo("not near obstacles")
                    self.avoiding = False
                    vel.linear.x = self.velMsg.linear.x
		    vel.angular.z = self.velMsg.angular.z
                    self.velPub.publish(vel)
            rate.sleep()

if (__name__ == "__main__"):
    ns = {}
    ns["sub"] = {}
    ns["pub"] = {}
    ns["sub"]["control"] = "/lurch_control"
    ns["sub"]["sonar"] = "/RosAria/sonar"
    ns["sub"]["cmd_vel_avoid"] = "/cmd_vel_avoid"
    ns["pub"]["cmd_vel"] = "/RosAria/cmd_vel"
    ns["pub"]["control"] = "/lurch_control"

    ca = CollisionAvoidance(ns)
    ca.collision_avoidance(100)
