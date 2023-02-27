import requests
from src.services.serviceUtil import serviceUtil

class InfoService():
    
	def __init__(self, token, api_url):
		self.token = token
		self.api_url = api_url
		
	def getInfo(self):
		headers = serviceUtil(self.token).makeAuthHeader()
		response = requests.get(self.api_url + "/info", headers=headers)
		return response.json()