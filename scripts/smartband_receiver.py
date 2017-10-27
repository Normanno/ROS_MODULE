#!/usr/bin/env python

import rospy
import tornado
import signal
import sys
from tornado_smartband_handler import RosSmartbandHandler
from base_class import Base
from base_class import topics as base_topics
from sgr_project.msg import SmartbandSensors
from tornado import websocket, web, httpserver, ioloop
from tornado.options import define, options

define('port', default=5092, help="ROS_PORT", type=int)


class SmartbandReceiver(Base):

    def __init__(self, topics):
        super(SmartbandReceiver, self).__init__()
        self.smartband_pub = rospy.Publisher(topics["publishers"]["smartband"], SmartbandSensors, queue_size=20)
        rospy.init_node('smrtband', anonymous=True)

    def simulate(self, rate):
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

    def parse(self, message):
        print 'parsing : '+str(message)
        msg = SmartbandSensors()
        msg.hr = int(message["hr"])
        msg.accx = float(message["acc_x"])
        msg.accy = float(message["acc_y"])
        msg.accz = float(message["acc_z"])
        msg.gyrox = float(message["gyro_x"])
        msg.gyroy = float(message["gyro_y"])
        msg.gyroz = float(message["gyro_z"])
        return msg

    def parse_and_publish(self, message):
        if rospy.is_shutdown():
            return
        msg = self.parse(message)
        self.smartband_pub.publish(msg)


class RosApp:

    def __init__(self):
        self.handlers = [
            (r'/smartband', RosSmartbandHandler)
        ]
        #tornado.web.Application.__init__(handlers=handlers)

    def make_app(self):
        print ' making app '+str(self.handlers)
        return tornado.web.Application(self.handlers)

    def stop(self):
        print 'stop ops'


def signal_handler(signal, frame):
    ws.stop()
    print 'Handling '+str(signal)+' signal'
    sys.exit(0)


if __name__ == '__main__':
    try:
        sm = SmartbandReceiver(base_topics)
        RosSmartbandHandler.smartband_publisher = sm
        #sm.simulate(10)
        rosApp = RosApp()
        ws = rosApp.make_app()
        ws.listen(options.port)
        print '**** Listening on port: '+str(options.port) + " ****"
        tornado.ioloop.IOLoop.current().start()
        signal.signal(signal.SIGINT, signal_handler)

    except rospy.ROSInterruptException:
        print 'Smartband sensing finished!'