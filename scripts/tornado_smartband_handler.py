import tornado
import tornado.web
import json


class RosSmartbandHandler(tornado.web.RequestHandler):

    smartband_publisher = None

    def check_and_publish(self):
        #data = json.loads(self.request.body.decode('utf-8'))
        data = json.loads(self.request.body)
        print "JSON MESSAGE: " + str(data)
        print data["uname"]
        print data["pass"]
        if data["uname"] == "ROS" and data["pass"] == "SGRA_ROS":
            if self.smartband_publisher is not None:
                self.smartband_publisher.parse_and_publish(data)
            else:
                print '**** ERROR: NO PUBLISHER SELECTED ****'
        else:
            print '**** ERROR: AUTHENTICATION FAILED ****'

    def get(self):
        print "get method"
        self.check_and_publish()

    def post(self):
        print "post method"
        self.check_and_publish()


    def head(self):
        print "head method"
