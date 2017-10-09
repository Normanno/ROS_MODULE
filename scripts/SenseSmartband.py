import rospy
import base_class
from base_class import topics
from sgr_project.msg import SmartbandSensors


class SenseSmartband(base_class):

    def __init__(self, topics):
        super(SenseSmartband, self).__init__()
        self.smartband_pub = rospy.Publisher(topics['publishers']['/sgra/smartband'])
        rospy.init_node('smrtbandsensor')

    def sense(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #DATI DI PROVA
            #TODO leggere da smartband
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
        sm = SmartbandSensors(topics)
    except rospy.ROSInterruptException:
        print 'Smartband sensing finished!'