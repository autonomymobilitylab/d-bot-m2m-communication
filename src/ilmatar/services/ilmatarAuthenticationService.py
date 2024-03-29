import requests

from src.api.oauth2Token import Oauth2Token

class IlmatarAuthenticationService():
	
	def __init__(self, user):
		self.user = user
		self.token = ""
		self.url = "TODO: Tähän oikea urli"

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
		print("status: ")
		print(response.status_code)
		resJson = response.json()
		self.token = Oauth2Token(resJson["expires_in"], resJson["access_token"], resJson["token_type"], resJson["scope"])
		return self.token
