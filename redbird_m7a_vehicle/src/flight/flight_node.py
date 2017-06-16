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
		robotArray = list()
		self._loc_map = Map()
		self._sim_map = Map()
		firstTime = True
		numRobots = 14
		priority = None
		attempts = 0
		successful = False

        # Initialize vehicle for tracking
        self._vehicle = Vehicle()
		
		# Create subscribers
		self._loc_sub = rospy.Subscriber("localization", Map, self.update_loc_map)
		self._loc_sub = rospy.Subscriber("simulation", Map, self.update_sim_map)

	def update_loc_map(self, data):
		self._loc_map = data
		
	def update_sim_map(self, data):
		self._sim_map = data
		
	def getPriorityByConfidence():
		tempConfidence = -1.0
		tempPriority = self._sim_map.target_robots[0]
		for robot in self._sim_map.target_robots:
			if (robot.confidence > tempConfidence):
				tempPriority = robot
				tempConfidence = robot.confidence
		return tempPriority	

	def distanceToRobot(robot):
		x = robot.x - self._vehicle.x
		y = robot.y - self._vehicle.y
		z = x**2 + y**2
		z = z**(1/2.0)	
		
	def robotOrientedCorrectly():
		#
		return True
		
	def flyTo(x, y, z):
		#fly to a location
	
    def start(self):
        while not rospy.is_shutdown():
			# rospy.loginfo(self._loc_map.target_robots[0].color)
			
			if (firstTime == True):
				#initialize system
				robotArray = self._sim_map.target_robots
				flyTo(0, 0, 2.5)
				firstTime = False
			
			if (numRobots > 0):
				priority = getPriorityByConfidence()
				attempts = 0
				if (priority.out_of_bounds == True):
						numRobots=numRobots-1
						robotArray.remove(priority)
				else:
					while (priority.out_of_bounds == False):
						while (distanceToRobot(priority) >= 0.5):
							flyTo(priority.x, priority.y, 2.5)
						if (priority.out_of_bounds == True):
							numRobots=numRobots-1
							robotArray.remove(priority)
							break
						else:
							if (robotOrientedCorrectly(priority) == True):
								# do nothing
							else:
								#
				if (successful == True):
					numRobots=numRobots-1
					robotArray.remove(priority)

if __name__ == '__main__':
    try:
        # Initialize node
        node = Flight()

        # Start node,
        node.start()
    except rospy.ROSInterruptException:
        pass
