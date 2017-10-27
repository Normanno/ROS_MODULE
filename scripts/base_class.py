#!/usr/bin/env python

import signal
import sys

topics = {
    'subscribers': {
        'smartband': '/sgra/smartband',
        'odometry': '/tracker/markers_array_smoothed',
        'personality': '/sgra/personality'
    },
    'publishers': {
        'smartband': '/sgra/smartband',
        'velocity': '/RosAria/cmd_vel',
        'personality': '/sgra/personality'
    }
}


# Base class for common basic operations
class Base(object):

    def __init__(self):
        # SIGINT handler
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signal, frame):
        self.stop()
        sys.exit(0)

    def stop(self):
        '''Operations to be performed on stop'''