import rospy
from std_msgs.msg import String
from ilmatar_python_lib import Crane
from dotenv import dotenv_values

from resources.config_loader import ConfigLoader


class CraneCommunication:
    def __init__(self, opcua_url):
        self.crane = Crane(opcua_url)
    
    def get_crane_hook_pos(self):
        return self.crane.get_coordinates_absolute()




if __name__ == '__main__':
    config = dotenv_values("resources/.env")
    if bool(config) == False:
        config = ConfigLoader()
        config.load(['CRANE_OPCUA_URL'])
    crane = CraneCommunication(config['CRANE_OPCUA_URL'])
    rospy.init_node('crane_communication')
    