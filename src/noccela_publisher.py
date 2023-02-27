import rospy
from std_msgs.msg import String

from src.noccela.api.status import Status, StatusSchema 
from src.noccela.services.noccelaAuthenticationService import NoccelaAuthenticationService
from src.noccela.services.serviceUtil import serviceUtil
from src.noccela.services.infoService import InfoService
from src.noccela.services.deviceService import DeviceService

import os

def publisher():
	pub = rospy.Publisher('noccela', String, queue_size=0)
	rate = rospy.Rate(16)
	msg_pub = String()

if __name__ == '__main__':		
	rospy.init_node('noccela_publisher')
	publisher()