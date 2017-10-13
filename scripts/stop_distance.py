#!/usr/bin/env python
import rospy
import pycurl
from StringIO import StringIO
from lxml import etree as ET
from sgr_project.msg import SmartbandSensors
from sgr_project.srv import ComputeStopDistance
from std_msgs.msg import *


class StopDistance(object):

    def __init__(self, topics):
        super(StopDistance, self).__init__()
        rospy.init_node('compute_edit_distance_server')
        rospy.Service('compute_edit_distance', ComputeStopDistance, self.compute)
        self.stop_dist_pub = rospy.Publisher(topics['publishers']['stopdistance'], Float32)
        self.smartband_sub = rospy.Subscriber(topics['subscribers']['smartband'], SmartbandSensors, 10)
        self.matlab_server_host = '127.0.0.1'
        self.matlab_server_port = '5902'

    def compute(self, req):

        root = ET.Element('root')
        ET.SubElement(root, "data", index="1", type="float", description="Acc.x").text = req.accx
        ET.SubElement(root, "data", index="2", type="float", description="Acc.y").text = req.accy
        ET.SubElement(root, "data", index="3", type="float", description="Acc.z").text = req.accz
        ET.SubElement(root, "data", index="4", type="float", description="Gyro.x").text = req.gyrox
        ET.SubElement(root, "data", index="5", type="float", description="Gyro.y").text = req.gyroy
        ET.SubElement(root, "data", index="6", type="float", description="Gyro.z").text = req.gyroz
        ET.SubElement(root, "data", index="7", type="int", description="Hr").text = req.hr
        ET.SubElement(root, "data", index="8", type="int", description="Velocity").text = req.velocity
        ET.SubElement(root, "data", index="9", type="int", description="Extraversion").text = req.extraversion
        ET.SubElement(root, "data", index="10", type="int", description="Agreebleness").text = req.agreebleness
        ET.SubElement(root, "data", index="11", type="int", description="Concientiounsness").text = req.concientiounsness
        ET.SubElement(root, "data", index="12", type="int", description="Neuroticism").text = req.neuroticism
        ET.SubElement(root, "data", index="13", type="int", description="Openness").text = req.openness

        tree = ET.tostring(ET.ElementTree(root))
        #s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #s.connect(self.matlab_server_host, self.matlab_server_port)



        #TODO return ComputeStopDistance( chiamata al servizio matlab)
        return ComputeStopDistance()

    def ws_request(self,xml_string):
        inputs = ET.tostring(ET.parse('test_files/file.xml'))
        ret_buffer = StringIO()
        c = pycurl.Curl()
        c.setopt(c.POST, 1)
        c.setopt(c.URL, self.matlab_server_host + ':' + self.matlab_server_port + '/computestopdistance')
        c.setopt(c.HTTPPOST, [("xml_data", xml_string)])
        c.setopt(c.WRITEFUNCTION, ret_buffer.write)
        c.perform()
        c.close()

        resp = ET.fromstring(ret_buffer.getvalue())
        sd = ET.SubElement(resp, 'StopDistance')


if __name__ == '__main__':
    print 'starting stop_distance_calculator node'