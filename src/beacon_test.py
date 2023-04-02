from noccela.services.noccelaAuthenticationService import NoccelaAuthenticationService
from noccela.services.infoService import InfoService
from dotenv import dotenv_values


class BeaconTest:
    def __init__(self, config):
        self.config = config
        self.authService = NoccelaAuthenticationService()
        self.token = self.authService.requestNoccelaAccessToken(
            config['BEACON_CLIENT_ID'], config['BEACON_CLIENT_SECRET'])
        self.info_service = InfoService(self.token, config['BEACON_API_url'])


if __name__ == '__main__':
    config = dotenv_values(".env")
    tester = BeaconTest(config)
