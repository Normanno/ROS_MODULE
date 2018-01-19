from xml.etree import ElementTree as ET
from xml.dom import minidom
import os.path


class Logger:

    def __init__(self, log_dir='~/logs/'):
        self.log_directory = log_dir

        if log_dir[len(log_dir)-1:] != '/':
            self.log_directory += '/'
        print 'current workdir ' + str(os.getcwd())
        if not os.path.exists(self.log_directory):
            os.makedirs(self.log_directory)

    def user_log_dir_path(self, uid):
        return self.log_directory + "user_" + str(uid) + "_logs/"

    def user_session_log_file_path(self, uid):
        return self.user_log_dir_path(uid) + str(uid) + "_logs.xml"

    def user_smartband_log_file_path(self, uid):
        return self.user_log_dir_path(uid) + str(uid) + '_band.csv'

    def init_user_logs(self, uid, personality):
        user_log_dir = self.user_log_dir_path(uid)
        if not os.path.exists(user_log_dir):
            os.makedirs(user_log_dir)
        root = ET.Element("exmperiments")
        info = ET.SubElement(root, "user-info")
        user = ET.SubElement(info, "uid")
        user.text = str(uid)
        personality_tracts = ET.SubElement(info, "personality")
        for key in personality.keys():
            tract = ET.SubElement(personality_tracts, str(key))
            tract.text = str(personality[key])
        sessions = ET.SubElement(root, "sessions")
        sessions.text = "\t"
        pretty_xml = minidom.parseString(ET.tostring(root)).toprettyxml(indent='\t')
        with open(self.user_session_log_file_path(uid), 'w+') as fx:
            fx.write(pretty_xml)
        open(self.user_smartband_log_file_path(uid), 'w+').close()

    def log_experiment(self, uid, timestamp_start, timestamp_end, start_at, stop_at, stop_distance, delta, autonomous):
        """
        >log_experiment(self, uid, start_at, stop_at, stop_distance, delta, deceleration, autonomous)
         Write all the data on an xml representing the user
        :param uid:
        :param timestamp_start
        :param timestamp_end
        :param start_at:
        :param stop_at:
        :param stop_distance:
        :param delta:
        :param autonomous:
        :return:
        """
        log_file = self.user_session_log_file_path(uid)
        if not os.path.exists(log_file):
            print 'Error no log file for the uid: '+str(uid)
            exit(1)
        tree = ET.parse(log_file)
        root = tree.getroot()
        sessions = root.find('sessions')
        actual_session = ET.SubElement(sessions, 'session')
        stop_distance_el = ET.SubElement(actual_session, 'stop-distance')
        distance = ET.SubElement(stop_distance_el, 'computed-distance')
        distance.text = str(stop_distance)
        delta_el = ET.SubElement(stop_distance_el, 'computed-delta')
        delta_el.text = str(delta)
        time_s = ET.SubElement(actual_session, 'starts-at-time')
        time_s.text = str(timestamp_start)
        time_e = ET.SubElement(actual_session, 'stops-at-time')
        time_e.text = str(timestamp_end)
        dist_s = ET.SubElement(actual_session, 'starts-at-distance')
        dist_s.text = str(start_at)
        dist_e = ET.SubElement(actual_session, 'stops-at-distance')
        dist_e.text = str(stop_at)
        pretty_xml = minidom.parseString(ET.tostring(root)).toprettyxml(indent="\t")
        with open(log_file, 'w') as fx:
            fx.write(pretty_xml)

    def log_smartband_data(self, uid, timestamp, smartband_data, velocity, stop_distance, delta, deceleration, human_reached):
        """
        >log_smartband_data(self, uid, smartband_data, stop_distance, delta, msg):
         write all the data in csv format with timestamp
        :param uid:
        :param smartband_data:
        :param stop_distance:
        :param delta:
        :param human_reached:
        :return:
        """
        log_file = self.user_smartband_log_file_path(uid)
        if not os.path.exists(log_file):
            print 'Error no log file for the uid: ' + str(uid)
            exit(1)
        s_smartband_data = "" + ";"
        for d in smartband_data:
            s_smartband_data += str(d) + ";"

        with open(log_file, 'a') as fc:
            fc.write(str(timestamp) + ";" + str(stop_distance) + ";" + str(delta) + ";" + s_smartband_data +
                     str(velocity) + ";" + str(deceleration) + ";" + str(human_reached))
