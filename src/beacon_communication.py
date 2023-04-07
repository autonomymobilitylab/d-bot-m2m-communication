import rospy
from std_msgs.msg import String
import asyncio
import websockets
import json
from dotenv import dotenv_values

from beacon.services.authenticationService import AuthenticationService
from beacon.services.websocketService import WebsocketService
from resources.config_loader import ConfigLoader

class BeaconCommunication:

    def __init__(self, config):
        self.authService = AuthenticationService(config['BEACON_BASE_AUTH_URL'], config['BEACON_AUTH_URL'])
        self.token = self.authService.requestAccessToken(
            config['BEACON_CLIENT_ID'], config['BEACON_CLIENT_SECRET'])
        if (self.token):
            rospy.loginfo('Received access token')
        self.beacon_ws = WebsocketService(self.token, config['BEACON_API_URL'], config['BEACON_ACCOUNT'], config['BEACON_SITE'])
        if (self.beacon_ws.ws_domain):
            rospy.loginfo('Received websocket domain location')
        self.rate = rospy.Rate(16)

    def taglocationPublisher(self):
        pub = rospy.Publisher('taglocation', String, queue_size=0)
        asyncio.get_event_loop().run_until_complete(self.fetchTagLocation(pub))

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
        config.load(['BEACON_BASE_AUTH_URL', 'BEACON_AUTH_URL', 'BEACON_CLIENT_ID', 'BEACON_CLIENT_SECRET', 'BEACON_API_URL', 'BEACON_ACCOUNT', 'BEACON_SITE'])
    rospy.init_node('beacon_communication')
    rospy.loginfo('node initialized')
    beacon = BeaconCommunication(config)
    rospy.loginfo('Beacon communication loaded')
    rospy.loginfo('Starting tag publishing')
    beacon.taglocationPublisher()