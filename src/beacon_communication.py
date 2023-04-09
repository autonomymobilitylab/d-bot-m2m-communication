import rospy
from std_msgs.msg import String
import asyncio
import websockets
import json
from dotenv import dotenv_values

from api.task import Task
from beacon.services.authenticationService import AuthenticationService
from beacon.services.websocketService import WebsocketService
from beacon.services.infoService import InfoService
from beacon.services.deviceService import DeviceService
from beacon.buzzerUtil import BuzzerUtil
from resources.config_loader import ConfigLoader
from d_bot_m2m_communication.srv import TaskCall, TaskCallResponse, TaskCallRequest

class BeaconCommunication:

    def __init__(self, config):
        self.authService = AuthenticationService(config['BEACON_BASE_AUTH_URL'], config['BEACON_AUTH_URL'])
        try:
            self.token = self.authService.requestAccessToken(
                config['BEACON_CLIENT_ID'], config['BEACON_CLIENT_SECRET'])
        except:
            rospy.loginfo('No connection to beacon auth')
        if (self.token):
            rospy.loginfo('Received access token')
        self.rate = rospy.Rate(rospy.get_param('publish_rate', 0.2))
        self.info_service = InfoService(self.token, config['BEACON_API_URL'])
        self.device_service = DeviceService(self.token, config['BEACON_API_URL'], config['BEACON_ACCOUNT'], config['BEACON_SITE'])
        try:
            self.beacon_ws = WebsocketService(self.token, config['BEACON_API_URL'], config['BEACON_ACCOUNT'], config['BEACON_SITE'])
            if (self.beacon_ws):
                rospy.loginfo('Received websocket domain location')
        except:
            rospy.loginfo('No connection to beacon websocket')
        self.config = config
        self.workarea_srv = self.start_workarea_protection_service()
        self.workarea = None
        self.workprotection = False

    def run_work_area_protection(self, area_id):
        #history = self.device_service.getTagHistoryAll()
        history = self.device_service.getTagHistory(config['BEACON_TEST_TAG_ID'])
        # rospy.loginfo(history)
        res = BuzzerUtil().get_recent_tags_in_workarea(history, area_id)
        for tag in res:
            rospy.loginfo('Found Tags in work area')
            success = self.device_service.play_tag_buzzer(tag)
            if success:
                rospy.loginfo("Playing tag buzzer")
            # hardcoded assumption area contains crane
            # TODO find good way to share enum definitions between packages
            task = Task(11,1)
            client = self.get_add_task_client(task)


    def init_work_area_protection(self, req):
        task = Task().load(req.task)
        try:
            self.workarea = task.device_id
            self.workprotection = True
            task.success = True
            rospy.loginfo('Starting work area protection')
        except:
            rospy.loginfo('Beacon workarea protection failed')
            task.error = "Beacon workarea protection failed"
            task.success = False
        task_json = task.jsonify()
        response = TaskCallResponse()
        response.task = task_json
        return response

    def get_add_task_client(self, task:Task):
        task_json = task.jsonify()
        rospy.wait_for_service('/task_manager/add_task')
        service_proxy = rospy.ServiceProxy('/task_manager/add_task', TaskCall)
        request = TaskCallRequest()
        request.task = task_json
        return service_proxy(request)

    def taglocationPublisher(self):
        pub = rospy.Publisher('taglocation', String, queue_size=0)
        asyncio.get_event_loop().run_until_complete(self.fetchTagLocation(pub))
    
    def start_workarea_protection_service(self):
        return rospy.Service('/beacon_communication/protection', TaskCall, self.init_work_area_protection)

    async def fetchTagLocation(self, pub):
        msg_pub = String()
        async with websockets.connect(self.beacon_ws.ws_url) as ws_connection:
            if self.beacon_ws.ws_connected == False:
                await self.beacon_ws.authenticate(ws_connection)

            # Subscribe to location updates
            if (self.beacon_ws.ws_subscribed == False):
                msg = {
                    "uniqueId": "getLocations",
                    "action": "registerTagLocation",
                    "payload": {}
                }
                msg = json.dumps(msg)
                await ws_connection.send(msg)
                self.ws_subscribed = True
            result = await ws_connection.recv()
            if self.beacon_ws.isPingMsg(result):
                await ws_connection.send('1')
                rospy.loginfo('PingPong')
            else:
                result = json.loads(result)
            while not rospy.is_shutdown():
                result = await ws_connection.recv()
                if result == '':
                    await ws_connection.send('1')
                    rospy.loginfo('PingPong')
                else:
                    result = json.loads(result)
                    rospy.loginfo('received location')
                    # msg = {} might need localization
                    # msg_pub.data = json.dumps(msg)
                    msg_pub = result
                    pub.publish(msg_pub)
                    rospy.loginfo(msg_pub)
                self.rate.sleep()

if __name__ == '__main__':
    config = dotenv_values("resources/.env")
    if bool(config) == False:
        config = ConfigLoader()
        config.load(['BEACON_BASE_AUTH_URL', 'BEACON_AUTH_URL', 'BEACON_CLIENT_ID', 'BEACON_CLIENT_SECRET', 'BEACON_API_URL', 'BEACON_ACCOUNT', 'BEACON_SITE', 'BEACON_TEST_TAG_ID', 'BEACON_TEST_AREA_ID'])
    rospy.init_node('beacon_communication')
    rospy.loginfo('node initialized')
    beacon = BeaconCommunication(config)
    rospy.loginfo('Beacon communication loaded')
    rospy.loginfo('Starting tag publishing')
    # beacon.taglocationPublisher()
    while not rospy.is_shutdown():
        if (beacon.workprotection):
            beacon.run_work_area_protection(beacon.workarea)
        beacon.rate.sleep()