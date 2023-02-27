class Oauth2Token():
    
	def __init__(self, expires_in, access_token, token_type, scope):
		self.expires_in = expires_in
		self.access_token = access_token
		self.token_type = token_type
		self.scope = scope

	def getTokenExpiresIn(self):
		return self.expires_in

	def getToken(self):
		return self.access_token
	
	def getTokenType(self):
		return self.token_type

	def getTokenScope(self):
		return self.scope