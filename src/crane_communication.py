import rospy
from std_msgs.msg import String
from ilmatar_python_lib.crane import Crane
from dotenv import dotenv_values

from d_bot_m2m_communication.srv import Position, PositionResponse
from d_bot_m2m_communication.srv import Status, StatusResponse
from d_bot_m2m_communication.srv import CraneTaskCall, CraneTaskCallResponse
from d_bot_m2m_communication.srv import Task, TaskResponse
from resources.config_loader import ConfigLoader


class CraneCommunication:
    def __init__(self, opcua_url):
        try:
            self.crane = Crane(opcua_url)
        except:
            rospy.loginfo('Crane connection failed')
        self.crane_pos_srv = self.start_crane_position_service()
        self.crane_status_srv = self.start_crane_status_service()
        self.crane_stopper_srv = self.start_crane_stopper_service()
    
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

    def stop_crane(self, req):
        # TODO stop correct device with request device_id
        self.crane.connect()
        self.crane.stop_all()
        self.crane.disconnect()
        return True

    # /crane_communication/position
    def start_crane_position_service(self):
        return rospy.Service('position', Task, self.get_crane_hook_pos)

    # /crane_communication/status
    def start_crane_status_service(self):
        return rospy.Service('status', Status, self.get_crane_movement_status)
    
    # /crane_communication/stop
    def start_crane_stopper_service(self):
        return rospy.Service('stop', CraneTaskCall, self.stop_crane)

if __name__ == '__main__':
    config = dotenv_values("resources/.env")
    if bool(config) == False:
        config = ConfigLoader()
        config.load(['CRANE_OPCUA_URL'])
    crane = CraneCommunication(config['CRANE_OPCUA_URL'])
    rospy.init_node('crane_communication')
    rospy.spin()
    