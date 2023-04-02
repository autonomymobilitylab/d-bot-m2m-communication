import requests
from oauthlib.oauth2 import BackendApplicationClient
from requests_oauthlib import OAuth2Session

class NoccelaAuthenticationService():
	
	def __init__(self, auth_base_url, token_url):
		self.token = ""
		self.auth_base_url = auth_base_url
		self.token_url = token_url

	def requestNoccelaAccessToken(self, client_id, client_secret):
		client = BackendApplicationClient(client_id=client_id)
		oauth = OAuth2Session(client=client)
		
		self.token = oauth.fetch_token(token_url=self.token_url, client_id=client_id,
			client_secret=client_secret)

		return self.token
	
	# Optional TODO
	# def refreshToken 

