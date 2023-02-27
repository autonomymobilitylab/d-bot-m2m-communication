import rospy
from std_msgs.msg import String

from src.kone.api.status import Status, StatusSchema 

import os

def publisher():
	pub = rospy.Publisher('kone', String, queue_size=0)
	rate = rospy.Rate(16)
	msg_pub = String()

if __name__ == '__main__':		
	rospy.init_node('kone_publisher')
	publisher()