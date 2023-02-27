import requests

from src.api.oauth2Token import Oauth2Token

class KoneAuthenticationService():
	
	def __init__(self, user):
		self.user = user
		self.token = ""
		self.url = "https://dev.kone.com"

	def requestKoneAccessToken(self, client_id, client_secret):
		address = self.url + "/api/v2/oauth2/token"
		response = requests.post(
			address,
			data={
				"grant_type": "client_credentials", 
				"scope": "application/inventory callgiving/* equipmentstatus/*"
			},
			auth=(client_id, client_secret),
		)
		resJson = response.json()
		self.token = Oauth2Token(resJson["expires_in"], resJson["access_token"], resJson["token_type"], resJson["scope"])
		return self.token
