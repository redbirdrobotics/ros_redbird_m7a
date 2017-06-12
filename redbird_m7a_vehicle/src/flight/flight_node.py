#!/usr/bin/python

import rospy
import time
from utils import Vehicle
from redbird_m7a_msgs.msg import Map, GroundRobotPosition

class Flight(object):
    def __init__(self):
        # Initialize node
        rospy.init_node('flight_node', anonymous=True)
		
		# Initialize variables
		self._loc_map = Map()

        # Initialize vehicle for tracking
        self._vehicle = Vehicle()
		
		# Create localization subscriber
		self._loc_sub = rospy.Subscriber("localization", Map, self.update_loc_map)

	def update_loc_map(self, data):
		self._loc_map = data
	
    def start(self):
        while not rospy.is_shutdown():
			rospy.loginfo(self._loc_map.target_robots[0].color)
            

if __name__ == '__main__':
    try:
        # Initialize node
        node = Flight()

        # Start node
        node.start()
    except rospy.ROSInterruptException:
        pass
		//hi
