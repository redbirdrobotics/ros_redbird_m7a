#!/usr/bin/python

import rospy
import time
from utils import Vehicle, Flight_Controller, Flight_Mode
from redbird_m7a_msgs.msg import Map, GroundRobotPosition

class Flight(object):
    def __init__(self):
        # Initialize node
        rospy.init_node('flight_node', anonymous=True)

        # Initialize variables
        self._loc_map = Map()
        self._sim_map = Map()
        self._firstTime = True
        self._numGroundRobots = 10
        self._priorityRobot = None
        self._attempts = 0
        self._successful = False
        self._centerX = 0
        self._centerY = 0
        self._landX = 0
        self._landY = 0
        self._robotFacingGoalAtStart = 5
        self._minDroneToRobotDistance = 0.5
        self._lastLandSuccesful = False
        self._startTime = time.time()

        # Initialize vehicle for tracking
        self._vehicle = Vehicle()

        # Initialize flight controller
        self._flight_controller = Flight_Controller()

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
        tempPriority = None
        for robot in self._sim_map.target_robots:
            if (robot.confidence > tempConfidence and robot.out_of_bounds == False):
                tempPriority = robot
                tempConfidence = robot.confidence
        return tempPriority

    def getNumRobotsInBounds(self):
        count = 0
        for robot in self._sim_map.target_robots:
            if (robot.out_of_bounds == False):
                count+=1
        return count


    def getDistanceToGroundRobot(self, robot):
        x = robot.x - self._vehicle.x
        y = robot.y - self._vehicle.y
        z = x**2 + y**2
        z = z**(1/2.0)
        return z

    def priorityRobotOrientedCorrectly(self):
        print("placeholder")

    def getDroneNearPriorityRobot(self, x):
        while (self.distanceToGroundRobot(self._priorityRobot) >= x):
            self.flyTo(self._priorityRobot.x, self._priorityRobot.y, 2.5)

    def flyTo(self, x, y, z):
        print("placeholder")

    def flyToGround(self, x, y):
        print("placeholder")

    def orientGroundRobot(self, robot):
        print("placeholder")

    def landOnGroundRobot(self, x, y):
        print("placeholder")
        self._lastLandSuccesful = True

    #Make a function that constantly updates the ground robot array because robots leave bounds

    def start(self):
        if not rospy.is_shutdown():

            #Initialize robot if first run
            if (self._firstTime == True):
                self.flyTo(self._centerX, self._centerY, 2.5)
                rospy.loginfo(self.getFlightTag(self) + "Drone has reached the center.")
                self._firstTime = False

            #Complete Loop
            while (self.getNumRobotsInBounds() > 0):
                #Allocate the next priority ground robot
                if (self.getNumRobotsInBounds() == 10):
                    self._priorityRobot = self._robotFacingGoalAtStart
                else:
                    #Currently finding priority by simulation confidence
                    self._priorityRobot = self.getPriorityRobotByConfidence()

                #Fly drone near robot for the first time
                self.getDroneNearPriorityRobot(self._minDroneToRobotDistance)

                #Check to see if ground robot has crossed the goal line or gone out of bounds
                while (self._priorityRobot.out_of_bounds == False):
                    #Check to see if ground robot is oriented towards goal line
                    if (self.priorityRobotOrientedCorrectly() == False):
                        self.landOnGroundRobot(self._priorityRobot.x, self._priorityRobot.y)
                        self.flyTo(self._vehicle.x, self._vehicle.y, 2.5)
                        self._attempts+=1
                        if (self._attempts >= 5):
                            self._attempts = 0
                            self._priorityRobot = self.getPriorityRobotByConfidence()

                    #Fly drone near robot again
                    self.getDroneNearPriorityRobot(self._minDroneToRobotDistance)

                #End of while loop, a robot has crossed goal line or gone out of bounds

                self._attempts = 0

            #Done!
            rospy.loginfo(self.getFlightTag() + "There are no more ground robots left.")
            self.flyToGround(self._landX, self._landY)
            rospy.loginfo(self.getFlightTag() + "Successfully Landed.")
            return

if __name__ == '__main__':
    try:
        # Initialize node
        node = Flight()

        # Start node
        # node.start()
    except rospy.ROSInterruptException:
        pass
