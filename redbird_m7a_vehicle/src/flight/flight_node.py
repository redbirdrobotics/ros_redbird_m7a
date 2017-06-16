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
        self._groundRobotArray = list()
        self._loc_map = Map()
        self._sim_map = Map()
        self._firstTime = True
        self._numRobots = 14
        self._priorityRobot = None
        self._attempts = 0
        self._successful = False
        self._centerX = 0
        self._centerY = 0
        self._startTime = time.time()

        # Initialize vehicle for tracking
        self._vehicle = Vehicle()
        
        # Create subscribers
        self._loc_sub = rospy.Subscriber("localization", Map, self.update_loc_map)
        self._sim_sub = rospy.Subscriber("simulation", Map, self.update_sim_map)

    def getFlightTag(self):
        return "[flight_node] "
    
    def update_loc_map(self, data):
        self._loc_map = data
        
    def update_sim_map(self, data):
        self._sim_map = data
        
    def getPriorityRobotByConfidence(self):
        tempConfidence = -1.0
        tempPriority = self._sim_map.target_robots[0]
        for robot in self._sim_map.target_robots:
            if (robot.confidence > tempConfidence):
                tempPriority = robot
                tempConfidence = robot.confidence
        return tempPriority    

    def getDistanceToGroundRobot(self, robot):
        x = robot.x - self._vehicle.x
        y = robot.y - self._vehicle.y
        z = x**2 + y**2
        z = z**(1/2.0) 
        return z   
        
    def robotOrientedCorrectly(self):
        #
        return True
        
    def flyTo(self, x, y, z):
        #fly to a location
        return
    
    def start(self):
        while not rospy.is_shutdown():
            # rospy.loginfo(self._loc_map.target_robots[0].color)
            
            if (self._firstTime == True):
                #initialize system
                self._groundRobotArray = self._sim_map.target_robots
                self.flyTo(self._centerX, self._centerY, 2.5)
                rospy.loginfo(self.getFlightTag() + "Drone has reached the center.")
                self._firstTime = False
            
            if (self._numRobots > 0):
                self._priorityRobot = self._getPriorityRobotByConfidence()
                self._attempts = 0
                if (self._priorityRobot.out_of_bounds == True):
                        self._numRobots-=1
                        self._groundRobotArray.remove(self._priorityRobot)
                else:
                    while (self._priorityRobot.out_of_bounds == False):
                        while (self.distanceToGroundRobot(self._priorityRobot) >= 0.5):
                            self.flyTo(self._priorityRobot.x, self._priorityRobot.y, 2.5)
                        if (self._priorityRobot.out_of_bounds == True):
                            self._numRobots-=1
                            self._groundRobotArray.remove(self._priorityRobot)
                            break
                        else:
                            if (self.robotOrientedCorrectly(self._priorityRobot) == True):
                                # do nothing
                                print("placeholder")
                            else:
                                #
                                print("placeholder")
                                
                if (self.successful == True):
                    self._numRobots-=1
                    self._groundRobotArray.remove(self._priorityRobot)

if __name__ == '__main__':
    try:
        # Initialize node
        node = Flight()

        # Start node,
        node.start()
    except rospy.ROSInterruptException:
        pass
