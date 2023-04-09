import json

class Task():
    
	def __init__(self, task_type = None, priority = None, device_id=None, location=None, success=False, error=None, task_id = None):
		self.task_type = task_type
		self.priority = priority
		self.device_id = device_id
		self.location = location
		self.success = success
		self.error = error
		self.task_id = task_id
	
	def load(self, data):
		data = json.loads(data)
		self.task_type = data['task_type']
		self.priority = data['priority']
		self.device_id = data['device_id']
		self.location = data['location']
		self.success = data['success']
		self.error = data['error']
		self.task_id = data['task_id']
		return self

	def stringify_task(self):
		data_dict = self.__dict__
		return ', '.join([f"{key}='{value}'" for key, value in data_dict.items()])
	
	def jsonify(self):
		data_dict = self.__dict__
		return json.dumps(data_dict)
