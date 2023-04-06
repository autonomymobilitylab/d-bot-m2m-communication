import requests
import json
import websockets

from kone.api.elevatorCall import elevatorCall

class ElevatorService():

    def __init__(self, config):
        self.token = ""
        self.url = config['ELEVATOR_WS_URL']

    async def callElevator(self):
        async with websockets.connect(f"{self.url}/stream-v2?accessToken={self.token}", 'koneapi') as ws_connection:
            # await self.authenticate(ws_connection)
            msg = json.dumps(self.generateliftCallPayload(elevatorCall()))
            await ws_connection.send(msg)
            result = await ws_connection.recv()
            result = json.loads(result)
            return result

    async def get_elevator_status(self):
        async with websockets.connect(f"{self.url}/stream-v2?accessToken={self.token}", 'koneapi') as ws_connection:
            # await self.authenticate(ws_connection)
            msg = json.dumps(self.generate_lift_status_payload(elevatorCall()))
            await ws_connection.send(msg)
            result = await ws_connection.recv()
            result = json.loads(result)
            return result
    
    async def cancel_elevator_call(self):
        async with websockets.connect(f"{self.url}/stream-v2?accessToken={self.token}", 'koneapi') as ws_connection:
            # await self.authenticate(ws_connection)
            msg = json.dumps(self.generate_lift_call_cancel_payload(elevatorCall()))
            await ws_connection.send(msg)
            result = await ws_connection.recv()
            result = json.loads(result)
            return result

    async def authenticate(self, ws_connection=None):
        connection = ws_connection
        if (connection == None):
            async with websockets.connect(self.ws_url) as ws_connection:
                connection = await ws_connection

        try:
            await connection.send(self.token['access_token'])
            res = await connection.recv()
            res = json.loads(res)

            if res['uniqueId'] == 'authSuccess':
                self.ws_connected = True
                return self.ws_connected
        except:
            print("authentication failed")
        return False

    def generate_lift_call_payload(self, elevatorcall):
        return {
            type: 'lift-call-api-v2',
            buildingId: elevatorcall.building_id,
            callType: 'action',
            groupId: '1',
            payload: {
                request_id: 252390420,
                area: 3000,
                time: '2022-03-10T07:17:33.298515Z',
                terminal: 1,
                call: {
                    action: 2,
                    destination: 5000
                }
            }
        }

    def generate_lift_status_payload(self, elevatorcall):
        return {
            type: 'lift-call-api-v2',
            buildingId: elevatorcall.building_id,
            callType: 'action',
            groupId: '1',
            payload: {
                request_id: 252390420,
                area: 3000,
                time: '2022-03-10T07:17:33.298515Z',
                terminal: 1,
                call: {
                    action: 2,
                    destination: 5000
                }
            }
        }

    def generate_lift_call_cancel_payload(self, elevatorcall):
        return {
            type: 'lift-call-api-v2',
            buildingId: elevatorcall.building_id,
            callType: 'delete',
            groupId: '1',
            payload: {
                session_id: elevatorcall.session_id
            }
        }
