from ilmatar_python_lib.crane import Crane
from dotenv import dotenv_values
import time

from resources.config_loader import ConfigLoader


class CraneCommunication:
    def __init__(self, opcua_url):
        self.crane = Crane(opcua_url)
    
    # returns PositionResponse
    def get_crane_hook_pos(self):
        return self.crane.get_coordinates_absolute()

    # return StatusResponse
    def get_crane_movement_status(self):
        res = True
        self.crane.get_trolley_speed_request()
        if (self.crane.get_trolley_speed_request() == 0 and 
            self.crane.get_bridge_speed_request() == 0 and
            self.crane.get_hoist_speed_request() == 0):
            res = False
        return res

if __name__ == '__main__':
    config = dotenv_values("resources/.env")
    if bool(config) == False:
        config = ConfigLoader()
        config.load(['CRANE_OPCUA_URL'])
    crane = CraneCommunication(config['CRANE_OPCUA_URL'])
    for x in range(60):
        print(crane.get_crane_hook_pos())
        print(crane.get_crane_movement_status())
        time.sleep(1)
    crane.crane.disconnect()
