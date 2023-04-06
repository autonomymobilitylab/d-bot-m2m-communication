import asyncio
import websockets
import json
from dotenv import dotenv_values

from d_bot_m2m_task_executor.srv import TaskCall, TaskCallResponse
from resources.config_loader import ConfigLoader
from d_bot_m2m_communication.src.kone.services.authenticationService import KoneAuthenticationService

class ElevatorCommunication:
    def __init__(self, config):
        self.auth_service = KoneAuthenticationService(config)
        self.auth_service.requestKoneAccessToken(config['ELEVATOR_CLIENT_id'], config['ELEVATOR_CLIENT_SECRET'])

if __name__ == '__main__':
    config = dotenv_values("resources/.env")
    if bool(config) == False:
        config = ConfigLoader()
        config.load(['ELEVATOR_CLIENT_id', 'ELEVATOR_CLIENT_SECRET', 'ELEVATOR_API_URL'])
    elevator = ElevatorCommunication(config)
