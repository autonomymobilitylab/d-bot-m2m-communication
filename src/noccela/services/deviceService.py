import requests
import json
from ..services.serviceUtil import serviceUtil

class DeviceService():
    
	def __init__(self, token, api_url, account):
		self.token = token
		self.account = account
		self.api_url = api_url
		
	def getDevicesInfo(self):
		params = (
		    ('Account', self.account),
		    ('IncludeVersionInfo', False),
		)
		headers = serviceUtil(self.token).makeAuthHeader()
		response = json.dumps(requests.get(self.api_url + "/device", headers=headers, params=params).json())
		return response
