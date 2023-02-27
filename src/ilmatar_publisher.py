import rospy
from std_msgs.msg import String

from src.ilmatar.api.status import Status, StatusSchema 

import os

def publisher():
	pub = rospy.Publisher('ilmatar', String, queue_size=0)
	rate = rospy.Rate(16)
	msg_pub = String()

if __name__ == '__main__':		
	rospy.init_node('ilmatar_publisher')
	publisher()