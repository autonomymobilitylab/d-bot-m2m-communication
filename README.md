# dbot m2m communication
D-bot ROS machine-to-machine communication package


## installation
rename .env.sample to .env
change properties template values to actual values

## install dependencies
pip install requests requests_oauthlib
pip install websockets

## starting package
```bash
source /catkin_ws/devel/setup.bash
for example lauching task manager
roslaunch d_bot_m2m_communication communication.launch
```