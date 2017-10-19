#!/usr/bin/env python
import rospy
from StringIO import StringIO
from sgr_project.msg import SmartbandSensors
from sgr_project.srv import ComputeStopDistance
from std_msgs.msg import *
from base_class import topics as base_topics
import matlab.engine


class StopDistance(object):

    def __init__(self, topics):
        super(StopDistance, self).__init__()
        rospy.init_node('compute_stop_distance_server')
        rospy.Service('compute_stop_distance', ComputeStopDistance, self.compute)
        #self.stop_dist_pub = rospy.Publisher(topics['publishers']['stopdistance'], Float32)
        #self.smartband_sub = rospy.Subscriber(topics['subscribers']['smartband'], SmartbandSensors, 10)
        self.engine = matlab.engine.start_matlab()
        out = StringIO.StringIO()
        err = StringIO.StringIO()
        self.ret = self.engine.dec2hex(2 ** 60, stdout=out, stderr=err)
        rospy.spin()

    def compute(self, req):
        values = list()
        values.append(req.accx)
        values.append(req.accy)
        values.append(req.accz)
        values.append(req.gyrox)
        values.append(req.gyroy)
        values.append(req.gyroz)
        values.append(req.hr)
        values.append(req.velocity)
        values.append(req.extraversion)
        values.append(req.agreebleness)
        values.append(req.concientiounsness)
        values.append(req.neuroticism)
        values.append(req.openness)

        return self.engine.getStopDistanceSGR(values, nargout=1)


if __name__ == '__main__':
    print 'starting stop_distance_calculator node'
    sd = StopDistance(base_topics)
