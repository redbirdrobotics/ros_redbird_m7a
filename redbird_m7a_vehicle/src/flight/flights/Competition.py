#!/usr/bin/python

# TODO: Add header docstring

import rospy
import time
import flightsys
from redbird_m7a_msgs.msg import *

class Competition(flightsys.Flight, object):
    NAME = 'competition_flight'
    LOG_TAG = 'competition'

    def __init__(self):
        # Call super constructor
        super(Competition, self).__init__(name=self.NAME, log_tag=self.LOG_TAG)

        # Initialize variables
        self._redRobotArray_loc = RedRobotMap()
        self._greenRobotArray_loc = GreenRobotMap()
        self._redRobotArray_sim = RedRobotMap()
        self._greenRobotArray_sim = GreenRobotMap()
        self._goal = Goal()
        self._firstTime = True
        self._numGroundRobots = 10
        self._priorityRobot = None
        self._attempts = 0
        self._successful = False
        self._homeX = 0.0
        self._homeY = 0.0
        self._minDroneToRobotDistance = 0.5
        self._lastLandSuccesful = False
        self._startTime = time.time()

        # Create subscribers
        self._redRobotSub_loc = rospy.Subscriber("loc", RedRobotMap, self.update_redRobot_loc)
        self._greenRobotSub_loc = rospy.Subscriber("loc", GreenRobotMap, self.update_greenRobot_loc)
        self._redRobotSub_sim = rospy.Subscriber("sim", RedRobotMap, self.update_redRobot_sim)
        self._greenRobotSub_sim = rospy.Subscriber("sim", GreenRobotMap, self.update_greenRobot_sim)

    def update_redRobot_loc(self, msg):
        self._redRobotArray_loc = msg

    def update_greenRobot_loc(self, msg):
        self._greenRobotArray_loc = msg

    def update_redRobot_sim(self, msg):
        self._redRobotArray_sim = msg

    def update_greenRobot_sim(self, msg):
        self._greenRobotArray_sim = msg

    def getRobotArrayLoc(self):
        return self._redRobotArray_loc + self._greenRobotArray_loc

    def getRobotArraySim(self):
        return self._redRobotArray_sim + self._greenRobotArray_sim

    def getPriorityRobotByLocConfidence(self):
        tempConfidence = -1.0
        tempPriority = None
        for robot in self.getRobotArrayLoc():
            if (robot.confidence > tempConfidence and robot.out_of_bounds == False):
                tempPriority = robot
                tempConfidence = robot.confidence
        return tempPriority

    def getNumRobotsInBounds(self):
        count = 0
        for robot in self.getRobotArrayLoc():
            if (robot.out_of_bounds == False):
                count+=1
        return count

    def getDistanceToGroundRobot(self, robot):
        x = robot.x - self.vehicle.x
        y = robot.y - self.vehicle.y
        z = x**2 + y**2
        z = z**(1/2.0)
        return z

    def priorityRobotOrientedCorrectly(self):
        vec_x = self._priorityRobot.vec_x
        vec_y = self._priorityRobot.vec_y
        slope = vec_y / vec_x
        x_m = self._goal.x_m
        y_m = self._goal.y_m


    def getDroneNearPriorityRobot(self, x):
        while (self.distanceToGroundRobot(self._priorityRobot) >= x):
            # Fly to point [priority robot location]
            self.fly_to_point((self._priorityRobot.x, self._priorityRobot.y, 2.5))

    def landOnGroundRobot(self, x, y):
        print("need a complex landing function here")
        self._lastLandSuccesful = True

    def flyStraightUp(self):
        self.fly_to_point((self.vehicle.get_position_x(), self.vehicle.get_position_y(), 2.5))

    def start(self):
        try:
            if not rospy.is_shutdown():

                #Initialize robot if first run
                if (self._firstTime == True):

                    #Set relative center
                    self._homeX = self.vehicle.get_position_x()
                    self._homeY = self.vehicle.get_position_y()

                    # Wait a moment
                    self.sleep(2)

                    # Takeoff
                    self.takeoff(2.5)

                    # Fly to point [home]
                    self.fly_to_point((self._homeX, self._homeY, 2.5))

                    #Finish startup
                    self.loginfo("Drone has reached the center.")
                    self._firstTime = False

                #Complete Loop
                while (self.getNumRobotsInBounds() > 0):

                    #Allocate the next priority ground robot, currently finding priority by simulation confidence
                    self._priorityRobot = self.getPriorityRobotByLocConfidence()

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
                                self._priorityRobot = self.getPriorityRobotByLocConfidence()

                        #Fly drone near robot again
                        self.getDroneNearPriorityRobot(self._minDroneToRobotDistance)

                    #End of while loop, a robot has crossed goal line or gone out of bounds

                    self._attempts = 0

                #Done!
                self.loginfo("There are no more ground robots left.")

                # Fly to point [Home] location]
                self.fly_to_point((self._homeX, self._homeY, 2.5))

                # Land
                self.land()

                #Complete
                self.loginfo("Successfully Landed.")
                return

        except KeyboardInterrupt:
            pass
