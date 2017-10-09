#!/usr/bin/env python

import signal
import sys

# subscribers/publishers namespace
topics = {
    'subscribers': {
        'smartband': '/sgra/smartband',
    },
    'publishers': {
        'smartband': '/sgra/smartband',
        'stopdistance': '/sgra/stopdistance'
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
        '''Operations to be perdormed on stop'''