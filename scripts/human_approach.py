#!/usr/bin/env python

import rospy
from base_class import topics as base_topics
from sgr_project.srv import ComputeStopDistance
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
from sgr_project.msg import SmartbandSensors
from base_class import Base


class HumanApproach(Base):

    def __init__(self, topics):
        super(HumanApproach, self).__init__()
        self.topics = topics
        self.odometry_subscriber = rospy.Subscriber(self.topics["subscribers"]["odometry"], MarkerArray, self.process_odometry)
        self.smartband_subscriber = rospy.Subscriber(topics["subscribers"]["smartband"], SmartbandSensors, self.process_smartband, 10)
        self.velocity_publisher = rospy.Publisher(self.topics["publishers"]["velocity"], Twist)
        self.humans = 0
        self.human_detected = False
        self.human_reached = False
        self.smartband_connected = False
        self.pose = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.stop_distance = 1.0
        rospy.init_node("human_approach", anonymous=True)

    def process_odometry(self, odometry):
        self.humans = len(odometry.markers)
        if self.humans >= 1:
            print "++ humans detected: " + str(self.humans) + " ++"
            self.human_detected = True
            self.pose["x"] = odometry.markers[0].pose.position.x
            self.pose["y"] = odometry.markers[0].pose.position.y
            self.pose["z"] = odometry.markers[0].pose.position.z
        else:
            print "-- no humans detected --"
            self.human_detected = False
            self.pose["x"] = 0.0
            self.pose["y"] = 0.0
            self.pose["z"] = 0.0

    def process_smartband(self, sb_sensors):
        rospy.wait_for_service('compute_edit_distance')
        compute_stop_distance = rospy.ServiceProxy('compute_stop_distance', ComputeStopDistance)
        self.stop_distance = compute_stop_distance(sb_sensors)


    def approach(self):
        velocity = Twist()
        rate = rospy.Rate(10)
        #TODO delete this line it's only for test purposes
        self.smartband_connected = True
        print 'approach cycle started'
        while not rospy.is_shutdown():
            if self.smartband_connected and self.human_detected: #If human is tracked and smartband is connected
#               rospy.wait_for_service('compute_edit_distance')
#                compute_stop_distance = rospy.ServiceProxy('compute_stop_distance', ComputeStopDistance)
#               stop_distance = compute_stop_distance()
                #move toward human
                if self.pose["x"] > self.stop_distance:
                    print "**** Human detected: moving ****"
                   #TODO UNCOMMENT THIS LINE
                    velocity.linear.x = 0.25
                    self.human_reached = False
                else:
                    print "**** Human detected: reached ****"
                    velocity.linear.x = 0.0
                    self.human_reached = False
                #turn toward human
                #TODO uncommentTHIS LINE IT'S ONLY TO TEST PURPOSES
                velocity.angular.z = min(1.0, self.pose["y"]) * 0.5
            elif not self.human_detected:
                print "**** Human lost! ****"
                #TODO uncommentTHIS LINE IT'S ONLY TO TEST PURPOSES
                velocity.angular.z = 0.5 #turn around to look for a human

            self.velocity_publisher.publish(velocity)
            rate.sleep()


if __name__ == "__main__":
    try:
        ha = HumanApproach(base_topics)
        ha.approach()
    except rospy.ROSInterruptException:
        print "aproach interrupt exception"
