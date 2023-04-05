# dbot m2m communication
D-bot ROS machine-to-machine communication package


## installation

### Add secrets and other properties
rename .env.sample to .env
change properties template values to actual values

Or 
rename catkin_env.sh.sample to catkin_env.sh
change properties template values to actual values
```bash
cp catkin_env.sh /path/to/workspace/devel
source /path/to/workspace/devel/catkin_env.sh
```

Program does read configs which are also located in
~/.bashrc

## install dependencies
pip install requests requests_oauthlib
pip install python-dotenv  
pip install websockets

### Testing connections
```bash
cd src
python3 beacon_tester.py
```
## starting package
```bash
source /catkin_ws/devel/setup.bash
for example lauching task manager
roslaunch d_bot_m2m_communication communication.launch
```
