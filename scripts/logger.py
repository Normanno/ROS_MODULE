from xml.etree import ElementTree as ET
import os.path


class Logger:

    def __init__(self, log_dir):
        self.log_directory = log_dir

        if log_dir[len(log_dir)-1:] != '/':
            self.log_directory += '/'

        if not os.path.exists(self.log_directory):
            os.makedirs(self.log_directory)

    def log_experiment(self, uid, start_at, stop_at, stop_distance, delta, deceleration, autonomous):
        log_file = self.log_directory + 'user_' + str(uid) + '_log.xml'
        if not os.path.exists(log_file):
            print 'create file'

        print 'append experiment data'