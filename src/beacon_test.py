from beacon.services.authenticationService import AuthenticationService
from beacon.services.infoService import InfoService
from beacon.services.deviceService import DeviceService
from beacon.services.websocketService import WebsocketService
from beacon.buzzerUtil import BuzzerUtil
from resources.config_loader import ConfigLoader


import asyncio
from dotenv import dotenv_values


class BeaconTest:
    def __init__(self, config):
        self.config = config
        self.authService = AuthenticationService(config['BEACON_BASE_AUTH_URL'], config['BEACON_AUTH_URL'])
        self.token = self.authService.requestAccessToken(
            config['BEACON_CLIENT_ID'], config['BEACON_CLIENT_SECRET'])
        self.info_service = InfoService(self.token, config['BEACON_API_URL'])
        self.device_service = DeviceService(self.token, config['BEACON_API_URL'], config['BEACON_ACCOUNT'], config['BEACON_SITE'])
        self.beacon_ws = WebsocketService(self.token, config['BEACON_API_URL'], config['BEACON_ACCOUNT'], config['BEACON_SITE'])

if __name__ == '__main__':
    config = dotenv_values("resources/.env")
    if bool(config) == False:
        config = ConfigLoader()
        config.load(['BEACON_BASE_AUTH_URL', 'BEACON_AUTH_URL', 'BEACON_CLIENT_ID', 'BEACON_CLIENT_SECRET', 'BEACON_API_URL', 'BEACON_ACCOUNT', 'BEACON_SITE', 'BEACON_TEST_TAG_ID', 'BEACON_TEST_AREA_ID'])
    tester = BeaconTest(config)
    print('account info:')
    print(tester.info_service.getInfo())
    print('devices info:')
    print(tester.device_service.getDevicesInfo())
    print('devices status info:')
    print(tester.device_service.getDevicesStatusInfo())
    print('tag history:')
    print(tester.device_service.getTagHistoryAll())
    print('specific tag history:')
    history = tester.device_service.getTagHistory(config['BEACON_TEST_TAG_ID'])
    print(history)
    print('work area buzzer')
    res = BuzzerUtil().get_recent_tags_in_workarea(history, config['BEACON_TEST_AREA_ID'])
    print(res)
    for tag in res:
        print(tester.device_service.play_tag_buzzer(tag))
    print('devices locations:')
    print(asyncio.run(tester.beacon_ws.fetchData()))
