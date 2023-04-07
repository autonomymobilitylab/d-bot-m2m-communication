import rospy
from std_msgs.msg import String
from ilmatar_python_lib import Crane
from dotenv import dotenv_values

from d_bot_m2m_communication.srv import Position, PositionResponse
from d_bot_m2m_communication.srv import Status, StatusResponse
from resources.config_loader import ConfigLoader


class CraneCommunication:
    def __init__(self, opcua_url):
        self.crane = Crane(opcua_url)
        self.crane_pos_srv = self.start_crane_position_service()
        self.crane_status_srv = self.start_crane_status_service()
    
    # returns PositionResponse
    def get_crane_hook_pos(self, req):
        self.crane.connect()
        res = self.crane.get_coordinates_absolute()
        self.crane.disconnect()
        return res

    # return StatusResponse
    def get_crane_movement_status(self, req):
        self.crane.connect()
        res = True
        self.crane.get_trolley_speed_request()
        if (self.crane.get_trolley_speed_request() == 0 and 
            self.crane.get_bridge_speed_request() == 0 and
            self.crane.get_hoist_speed_request == 0):
            res = False
        self.crane.disconnect()
        return res

    def start_crane_position_service(self):
        return rospy.Service('position', Position, self.get_crane_hook_pos)

    def start_crane_status_service(self):
        return rospy.Service('status', Status, self.get_crane_movement_status)

if __name__ == '__main__':
    config = dotenv_values("resources/.env")
    if bool(config) == False:
        config = ConfigLoader()
        config.load(['CRANE_OPCUA_URL'])
    crane = CraneCommunication(config['CRANE_OPCUA_URL'])
    rospy.init_node('crane_communication')
    rospy.spin()
    