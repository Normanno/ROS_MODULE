import tornado.web
import json


class RosSmartbandHandler(tornado.web.RequestHandler):

    smartband_publisher = None

    def check_and_publish(self):
        data = json.loads(self.request.body)
        if data["uname"] == "ROS" and data["pass"] == "SGRA_ROS":
            if self.smartband_publisher is not None:
                self.smartband_publisher.parse_and_publish(data)
            else:
                print '**** ERROR: NO PUBLISHER SELECTED ****'
        else:
            print '**** ERROR: AUTHENTICATION FAILED ****'

    def get(self):
        self.check_and_publish()

    def post(self):
        self.check_and_publish()

    def head(self):
        print "Ping received"


class RosSmartphoneControl(tornado.web.RequestHandler):

    def get(self):
        print ''

    def head(self):
        print ''