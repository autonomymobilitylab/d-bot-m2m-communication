# dbot m2m communication
D-bot ROS machine-to-machine communication package


## installation

### Add secrets and other properties
rename .env.sample to .env
change properties template values to actual values

### Install dependencies
pip install requests requests_oauthlib  
pip install websockets

### Testing connections
```bash
cd src
python3 beacon_tester.py
```
