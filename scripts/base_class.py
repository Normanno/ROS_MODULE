#!/usr/bin/env python

import signal
import sys
import rosnode

topics = {
    'subscribers': {
        'smartband': '/sgra/smartband',
        'odometry': '/tracker/markers_array_smoothed',
        'personality_ctrl': '/sgra/control/personality',
        'approach': '/sgra/control/approach',
        'smartband_state': '/sgra/control/smartband_state',
        'stop_distance': '/sgra/stop_distance',
        'stop_distance_adapted': '/sgra/stop_distance_adapted',
        'velocity': '/RosAria/cmd_vel',
        'motors': '/RosAria/motors_state',
        'sonar': '/RosAria/sonar',
        'velocity_ctrl': '/sgra/control/velocity',
        'adaptation_ctrl': '/sgra/control/adaptation',
        'approach_ctrl': '/sgra/control/restart_approach',
        'human_reached': '/sgra/human_reached'
    },
    'publishers': {
        'smartband': '/sgra/smartband',
        'velocity': '/RosAria/cmd_vel',
        'approach': '/sgra/control/approach',
        'smartband_state': '/sgra/control/smartband_state',
        'stop_distance': '/sgra/stop_distance',
        'stop_distance_adapted': '/sgra/stop_distance_adapted',
        'personality_ctrl': '/sgra/control/personality',
        'velocity_ctrl': '/sgra/control/velocity',
        'adaptation_ctrl': '/sgra/control/adaptation',
        'approach_ctrl': '/sgra/control/restart_approach',
        'human_reached': '/sgra/human_reached'
    }
}


# Base class for common basic operations
class Base(object):

    def __init__(self):
        # SIGINT handler
        signal.signal(signal.SIGINT, self.signal_handler)
        nodes = rosnode.get_node_names()
        self.ros_aria_connected = True if '/RosAria' in nodes else False

    def signal_handler(self, signal, frame):
        self.stop()
        sys.exit(0)

    def stop(self):
        '''Operations to be performed on stop'''