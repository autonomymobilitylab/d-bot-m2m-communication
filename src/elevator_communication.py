import rospy
from std_msgs.msg import String
import asyncio
import websockets
import json
from dotenv import dotenv_values

from d_bot_m2m_task_executor.srv import TaskCall, TaskCallResponse
from resources.config_loader import ConfigLoader

class ElevatorCommunication:
    def __init__(self, config):
        self.rate = rospy.Rate(1)
        # TODO remove when services are done
        self.hello_pub = self.startHelloworldPublisher()

        self.call_service = self.startElevatorcallService()
        self.status_service = self.startElevatorStatusService()

    def startHelloworldPublisher(self):
        return rospy.Publisher('elevator_hello', String, queue_size=10)

    def startElevatorStatusService(self):
        return rospy.Service('status', TaskCall, self.getElevatorStatus)

    def startElevatorcallService(self):
        return rospy.Service('request', TaskCall, self.callElevator)

    def getElevatorStatus(self, req):
        rospy.loginfo('got service call to request elevator status')
        rospy.loginfo('Requesting elevator status')
        response = TaskCallResponse(req)
        return response

    def callElevator(self, req):
        rospy.loginfo('got service call to request elevator to a location')
        rospy.loginfo('Requesting elevator call')
        response = TaskCallResponse(req)
        return response

if __name__ == '__main__':
    config = dotenv_values("resources/.env")
    if bool(config) == False:
        config = ConfigLoader()
        config.load(['ELEVATOR_CLIENT_id', 'ELEVATOR_CLIENT_SECRET', 'ELEVATOR_API_URL'])
    rospy.init_node('elevator_communication')
    rospy.loginfo('node initialized')
    elevator = ElevatorCommunication(config)
    while not rospy.is_shutdown():
        message = 'Hello, world!'
        rospy.loginfo(message)
        # TODO remove when services are done
        elevator.hello_pub.publish(message)
        elevator.rate.sleep()
