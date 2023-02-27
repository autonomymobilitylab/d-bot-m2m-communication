import requests
import json
import websockets
from src.services.serviceUtil import serviceUtil

class WebsocketService():
	
	def __init__(self, token, api_url, acc, site):
		self.token = token
		self.api_url = api_url
		self.acc = 5109
		self.site = 1
		self.ws_domain = self.getWsDomain()
		self.ws_url = 'wss://'+ self.ws_domain + '/realtime?account={acc}&site={site}'.format(acc=self.acc, site=self.site) 
		self.ws_connected = False
		self.ws_connection = ""
	
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

	async def authenticate(self, ws_connection):
		await ws_connection.send(self.token['access_token'])

		res = await ws_connection.recv()
		res = json.loads(res)

		if res['uniqueId'] == 'authSuccess':
			self.ws_connected = True
		return self.ws_connected

	async def fetchData(self):	
		async with websockets.connect(self.ws_url) as ws_connection:
			self.ws_connection = ws_connection
			if self.ws_connected == False:
				self.authenticate(ws_connection)

			# Subscribe to location updates
			msg = {
				"uniqueId": "getLocations",
				"action": "registerTagLocation",
				"payload": {}
			}
			msg = json.dumps(msg)
			await ws_connection.send(msg)
			await ws_connection.recv()

	def isPingMsg(result):
		return result == ''
	
	async def asnwerPingmsg(self):
		await self.ws_connection.send('1')
