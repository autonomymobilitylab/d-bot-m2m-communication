import requests
import json
import websockets
import time

from ..services.serviceUtil import serviceUtil

class WebsocketService():

    def __init__(self, token, api_url, acc, site):
        self.token = token
        self.api_url = api_url
        self.acc = acc
        self.site = site
        self.ws_domain = self.getWsDomain()
        self.ws_url = 'wss://' + self.ws_domain + \
            '/realtime?account={acc}&site={site}'.format(
                acc=self.acc, site=self.site)
        self.ws_connected = False
        self.ws_connection = None
        self.ws_subscribed = False

    def getWsDomain(self):
        url = self.api_url + "/realtime/domain"
        headers = serviceUtil(self.token).makeAuthHeader()
        params = (
            ('Account', self.acc),
            ('Site', self.site),
        )
        response = requests.get(url, headers=headers, params=params)
        ws_domain = response.json()['domain']
        return ws_domain

    async def authenticate(self, ws_connection=None):
        connection = ws_connection
        if (connection == None):
            async with websockets.connect(self.ws_url) as ws_connection:
                connection = await ws_connection

        try:
            await connection.send(self.token['access_token'])
            res = await connection.recv()
            res = json.loads(res)

            if res['uniqueId'] == 'authSuccess':
                self.ws_connected = True
                return self.ws_connected
        except:
            print("authentication failed")
        return False

    async def fetchData(self):
        async with websockets.connect(self.ws_url) as ws_connection:
            if self.ws_connected == False:
                await self.authenticate(ws_connection)

            # Subscribe to location updates
            if (self.ws_subscribed == False):
                msg = {
                    "uniqueId": "123",
                    "action": "registerTagLocation",
                    "payload": {}
                }
                msg = json.dumps(msg)
                await ws_connection.send(msg)
                self.ws_subscribed = True
            result = await ws_connection.recv()
            if self.isPingMsg(result):
                # Respond with pong
                await ws_connection.send('1')
                print('PingPong')
            else:
                result = json.loads(result)
            return result

    def isPingMsg(self, result):
        return result == ''
