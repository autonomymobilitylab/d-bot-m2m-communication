import rospy
from std_msgs.msg import String
from ilmatar_python_lib.crane import Crane
from dotenv import dotenv_values

from api.task import Task
from d_bot_m2m_communication.srv import TaskCall, TaskCallResponse
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
    
    def get_crane_hook_pos(self, req):
        task = Task().load(req.task)
        rospy.loginfo(task.jsonify())
        try:
            self.crane.connect()
            res = self.crane.get_coordinates_absolute()
            self.crane.connect()
            if (res):
                task.location = {
                    "x": res[0],
                    "y": res[1],
                    "z": res[2]
                }
            self.crane.disconnect()
            task.success = True
        except:
            rospy.loginfo('Crane connection failed')
            task.error = "Crane connection failed"
            task.success = False
        task_json = task.jsonify()
        response = TaskCallResponse()
        response.task = task_json
        return response

    def get_crane_movement_status(self, req):
        task = Task().load(req.task)
        try:
            self.crane.connect()
            res = False
            if (self.crane.get_trolley_speed_request() == 0 and
                self.crane.get_bridge_speed_request() == 0 and
                self.crane.get_hoist_speed_request() == 0):
                res = True
                rospy.loginfo("crane not moving")
            self.crane.disconnect()
            task.success=res
        except:
            rospy.loginfo('Crane connection failed')
            task.error = "Crane connection failed"
            task.success = False
        task_json = task.jsonify()
        response = TaskCallResponse()
        response.task = task_json
        return response

    def stop_crane(self, req):
        # TODO stop correct device with request device_id
        task = Task().load(req.task)
        try:
            self.crane.connect()
            self.crane.stop_all()
            self.crane.disconnect()
            task.success = True
        except:
            rospy.loginfo('Crane connection failed')
            task.error = "Crane connection failed, no stopping"
            task.success = False
        task_json = task.jsonify()
        response = TaskCallResponse()
        response.task = task_json
        return response

    def start_crane_position_service(self):
        return rospy.Service('/crane_communication/position', TaskCall, self.get_crane_hook_pos)

    def start_crane_status_service(self):
        return rospy.Service('/crane_communication/status', TaskCall, self.get_crane_movement_status)
    
    def start_crane_stopper_service(self):
        return rospy.Service('/crane_communication/stop', TaskCall, self.stop_crane)

if __name__ == '__main__':
    config = dotenv_values("resources/.env")
    if bool(config) == False:
        config = ConfigLoader()
        config.load(['CRANE_OPCUA_URL'])
    crane = CraneCommunication(config['CRANE_OPCUA_URL'])
    rospy.init_node('crane_communication')
    rospy.spin()
    