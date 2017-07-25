#!/usr/bin/python

# TODO: Add header docstring

import rospy
import time
import flightsys
from flightsys import Controller, Control_Mode
from utils import Vehicle, Flight_Controller, Flight_Mode
from redbird_m7a_msgs.msg import Map, GroundRobotPosition


class Competition(flightsys.Flight, object):
    NAME = 'competition_flight'
    LOG_TAG = 'competition'

    def __init__(self):
        # Call super constructor
        super(Competition, self).__init__(name=self.NAME, log_tag=self.LOG_TAG)

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
        self._minDroneToRobotDistance = 0.5
        self._lastLandSuccesful = False
        self._startTime = time.time()

        # Initialize vehicle for tracking
        self._vehicle = Vehicle()

        # Initialize flight controller
        #??? self._flight_controller = Flight_Controller()

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
            # Fly to point [priority robot location]
            self.fly_to_point((self._priorityRobot.x, self._priorityRobot.y, 2.5))

    def landOnGroundRobot(self, x, y):
        print("need a complex landing function here")
        self._lastLandSuccesful = True

    def flyStraightUp(self):
        self.fly_to_point((self.vehicle.get_position_x(), self.vehicle.get_position_y(), 2.5))

    #Make a function that constantly updates the ground robot array because robots leave bounds

    def start(self):
        try:
            if not rospy.is_shutdown():

                #Initialize robot if first run
                if (self._firstTime == True):

                    #Set relative center
                    self._centerX = self.vehicle.get_position_x()
                    self._centerY = self.vehicle.get_position_y()

                    # Wait a moment
                    self.sleep(2)

                    # Takeoff
                    self.takeoff(2.5)

                    # Fly to point [center]
                    self.fly_to_point((self._centerX, self._centerY, 2.5))

                    #Finish startup
                    self.loginfo(self.getFlightTag(self) + "Drone has reached the center.")
                    self._firstTime = False

                #Complete Loop
                while (self.getNumRobotsInBounds() > 0):

                    #Allocate the next priority ground robot, currently finding priority by simulation confidence
                    self._priorityRobot = self.getPriorityRobotByConfidence()

                    #Fly drone near robot for the first time if still in bounds
                    if (self._priorityRobot.out_of_bounds == False):
                        self.getDroneNearPriorityRobot(self._minDroneToRobotDistance)

                    #Check to see if ground robot has crossed the goal line or gone out of bounds
                    while (self._priorityRobot.out_of_bounds == False):

                        #Check to see if ground robot is oriented towards goal line
                        if (self.priorityRobotOrientedCorrectly() == False):

                            #Land on the robot and rotate it
                            self.landOnGroundRobot(self._priorityRobot.x, self._priorityRobot.y)

                            # Fly to point [straight up]
                            self.flyStraightUp()

                            #Add an failed attempt every time you must rotate the robot
                            if (self._lastLandSuccesful == False):
                                self._attempts+=1

                            #If the drone has taken too long on a certain robot
                            if (self._attempts >= 3):

                                #Reset attempts and fetch a new priority robot
                                self._attempts = 0
                                self._priorityRobot = self.getPriorityRobotByConfidence()

                        #Fly drone near robot again
                        self.getDroneNearPriorityRobot(self._minDroneToRobotDistance)

                    #End of while loop, a robot has crossed goal line or gone out of bounds

                    self._attempts = 0

                #Done!
                self.loginfo(self.getFlightTag() + "There are no more ground robots left.")

                # Fly to point [landing location]
                self.fly_to_point((self._landX, self._landY, 2.5))

                # Land
                self.land()

                #Complete
                self.loginfo(self.getFlightTag() + "Successfully Landed.")
                return

        except KeyboardInterrupt:
            pass
