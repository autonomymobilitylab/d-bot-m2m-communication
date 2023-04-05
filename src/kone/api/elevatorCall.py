
class elevatorCall():
    def __init__(self, building_id = None, destination = None, action = None, request_id = None, session_id = None):
        self.building_id = building_id
        self.destination = destination
        self.action = action
        self.request_id = request_id

        # api session id, used to track call
        self.session_id = session_id
