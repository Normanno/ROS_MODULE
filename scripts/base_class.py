#!/usr/bin/env python

import signal
import sys
from sgr_project.msg import Releasers

# subscribers/publishers namespace
topics = {
    'subscribers': {
        'smartband': '/sgra/smartband',
        'odometry': '/tracker/markers_array_smoothed',
    },
    'publishers': {
        'smartband': '/sgra/smartband',
        'velocity': '/RosAria/cmd_vel',
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