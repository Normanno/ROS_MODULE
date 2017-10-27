#!/usr/bin/env python

import rospy
import StringIO
from sgr_project.srv import ComputeStopDistance
from base_class import topics as base_topics
import matlab.engine


engine = matlab.engine.start_matlab()
out = StringIO.StringIO()
err = StringIO.StringIO()
ret = engine.dec2hex(2 ** 60, stdout=out, stderr=err)


class StopDistance(object):

    def __init__(self, topics):
        super(StopDistance, self).__init__()
        rospy.init_node('compute_stop_distance_server')
        rospy.Service('compute_stop_distance', ComputeStopDistance, self.compute)
        rospy.spin()

    def compute(self, req):
        print 'computing stop distance'
        values = list()
        values.append(req.sddata.sbsensors.accx)
        values.append(req.sddata.sbsensors.accy)
        values.append(req.sddata.sbsensors.accz)
        values.append(req.sddata.sbsensors.gyrox)
        values.append(req.sddata.sbsensors.gyroy)
        values.append(req.sddata.sbsensors.gyroz)
        values.append(req.sddata.sbsensors.hr)
        values.append(req.sddata.velocity)
        values.append(req.sddata.extraversion)
        values.append(req.sddata.agreebleness)
        values.append(req.sddata.concientiouness)
        values.append(req.sddata.neuroticism)
        values.append(req.sddata.openness)
        return engine.getStopDistanceSGR(values, nargout=1)


if __name__ == '__main__':
    print 'starting stop_distance_calculator node'
    sd = StopDistance(base_topics)
