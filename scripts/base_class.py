#!/usr/bin/env python

import signal
import sys

topics = {
    'subscribers': {
        'smartband': '/sgra/smartband',
        'odometry': '/tracker/markers_array_smoothed',
        'personality_ctrl': '/sgra/control/personality',
        'control': '/sgra/control/main',
        'velocity': '/RosAria/cmd_vel',
        'motors': '/RosAria/motors_state',
        'sonar': '/RosAria/sonar',
        'velocity_ctrl': '/sgra/control/velocity',
        'human_reached': '/sgra/human_reached'
    },
    'publishers': {
        'smartband': '/sgra/smartband',
        'velocity': '/RosAria/cmd_vel',
        'control': '/sgra/control/main',
        'personality_ctrl': '/sgra/control/personality',
        'velocity_ctrl': '/sgra/control/velocity',
        'human_reached': '/sgra/human_reached'
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