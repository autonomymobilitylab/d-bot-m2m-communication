from marshmallow import Schema, fields

class Status():

	def __init__(self, isActive):
		self.status = isActive


	def getStatus():
		return self.status

class StatusSchema(Schema):
    status = fields.Boolean()