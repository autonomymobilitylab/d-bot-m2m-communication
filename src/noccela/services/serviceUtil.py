
class serviceUtil():
    
	def __init__(self, token):
		self.token = token
	
	def makeAuthHeader(self):
		return { 'Authorization': 'Bearer ' + self.token['access_token'],}
	
	def isTokenInvalidOrExpired(self):
		if self.token['access_token'] == "":
			return True
		return False