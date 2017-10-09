import rospy
from sgr_project.msg import SmartbandSensors

import base_class

class PersonApproach(base_class):

    def __init__(self, topics):
        super(PersonApproach, self).__init__()
        self.smartband_sensors = topics['subscribers']['smartband']

    def approach(self):
        '''Legge dati da smarband li correla coi dati della persona e richiede
            al servizio matlab l'inserimento'''
