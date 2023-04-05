from noccela.services.noccelaAuthenticationService import NoccelaAuthenticationService
from noccela.services.infoService import InfoService
from noccela.services.deviceService import DeviceService
from noccela.services.websocketService import WebsocketService
from resources.config_loader import ConfigLoader

import asyncio
from dotenv import dotenv_values


class BeaconTest:
    def __init__(self, config):
        self.config = config
        self.authService = NoccelaAuthenticationService(config['BEACON_BASE_AUTH_URL'], config['BEACON_AUTH_URL'])
        self.token = self.authService.requestNoccelaAccessToken(
            config['BEACON_CLIENT_ID'], config['BEACON_CLIENT_SECRET'])
        self.info_service = InfoService(self.token, config['BEACON_API_URL'])
        self.device_service = DeviceService(self.token, config['BEACON_API_URL'], config['BEACON_ACCOUNT'])
        self.beacon_ws = WebsocketService(self.token, config['BEACON_API_URL'], config['BEACON_ACCOUNT'], config['BEACON_SITE'])


if __name__ == '__main__':
    config = dotenv_values("resources/.env")
    if bool(config) == False:
        config = ConfigLoader()
        config.load(['BEACON_BASE_AUTH_URL', 'BEACON_AUTH_URL', 'BEACON_CLIENT_ID', 'BEACON_CLIENT_SECRET', 'BEACON_API_URL', 'BEACON_ACCOUNT', 'BEACON_SITE'])
    tester = BeaconTest(config)
    print('access_token:')
    print(tester.token)
    print('account info:')
    print(tester.info_service.getInfo())
    print('devices info:')
    print(tester.device_service.getDevicesInfo())
    print('devices locations:')
    print(asyncio.run(tester.beacon_ws.fetchData()))
