import requests
import json
from ..services.serviceUtil import serviceUtil
from ..services.websocketService import WebsocketService

class DeviceService():
    
	def __init__(self, token, api_url, account, site):
		self.token = token
		self.account = account
		self.api_url = api_url
		self.site = site
		
	def getDevicesInfo(self):
		params = (
		    ('Account', self.account),
		    ('IncludeVersionInfo', False),
		)
		headers = serviceUtil(self.token).makeAuthHeader()
		response = json.dumps(requests.get(self.api_url + "/device", headers=headers, params=params).json())
		return response

	def getDevicesStatusInfo(self):
		wsdomain= WebsocketService(self.token,self.api_url, self.account, self.site).ws_domain
		print('https://' + wsdomain)
		params = (
		    ('Account', self.account),
		    ('Site', self.site),
		)
		headers = serviceUtil(self.token).makeAuthHeader()
		response = json.dumps(requests.get('https://' + wsdomain + "/device/neighbors", headers=headers, params=params).json())
		return response

	def getTagHistoryAll(self):
		params = (
		    ('Account', self.account),
		    ('Site', self.site),
		    ('Count', 10),
		)
		headers = serviceUtil(self.token).makeAuthHeader()
		response = json.dumps(requests.get(self.api_url + "/history", headers=headers, params=params).json())
		return response

	def getTagHistory(self, tag_id = None):
		params = (
		    ('Account', self.account),
		    ('Site', self.site),
		    ('Count', 10),
		    ('Devices', [tag_id]),
		)
		headers = serviceUtil(self.token).makeAuthHeader()
		response = requests.get(self.api_url + "/history", headers=headers, params=params).json()
		return response

	def play_tag_buzzer(self, tag_id, buzzer_seconds = 15, led_seconds=15):
		data = {
			"account": self.account,
			"site": self.site,
			"devices": [tag_id],
			"alertsound": True,
			"buzzerSeconds": buzzer_seconds,
			"ledSeconds": led_seconds,
			"buzzerOnInterval": 5,
			"buzzerOffInterval": 5,
			"playWithDelay": False
		}
		data = json.dumps(data)
		headers={
 		   'Content-type':'application/json', 
		    'Accept':'application/json',
		    'Authorization': 'Bearer ' + self.token['access_token'],
			
		}
		res = requests.post(self.api_url + "/device/buzzer", headers = headers, data = data)
		if (res.status_code == 204):
			return True
		return False
