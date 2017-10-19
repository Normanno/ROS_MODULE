#!/usr/bin/env python

import rospy
from base_class import Base
from base_class import topics as base_topics
from sgr_project.msg import SmartbandSensors


class SmartbandReceiver(Base):

    def __init__(self, topics):
        super(SmartbandReceiver, self).__init__()
        self.smartband_pub = rospy.Publisher(topics["publishers"]["smartband"], SmartbandSensors, queue_size=20)
        rospy.init_node('smrtband', anonymous=True)

    def sense(self, rate):
        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            #DATI DI PROVA
            #TODO leggere da smartband
            print 'publishing message'
            msg = SmartbandSensors()
            msg.accx = -0.932617190000000
            msg.accy = -0.0603027300000000
            msg.accx = 0.442382810000000
            msg.gyrox = -66.7682926800000
            msg.gyroy = -14.2073170700000
            msg.gyroz = -41.3414634100000
            msg.hr = 79
            msg.velocity = 1
            self.smartband_pub.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        sm = SmartbandReceiver(base_topics)
        sm.sense(10)
    except rospy.ROSInterruptException:
        print 'Smartband sensing finished!'
