#!/usr/bin/python

"""Corvus1.py: Used for the first competition."""

import rospy
import time
import flightsys
from redbird_m7a_msgs.msg import *

__author__ = "Jonah Largen"

class Corvus1(flightsys.Flight, object):
    NAME = 'Corvus1'
    LOG_TAG = 'Corvus1'

    def __init__(self):
        # Call super constructor
        super(Corvus1, self).__init__(name=self.NAME, log_tag=self.LOG_TAG)

        #Initialize Variables
        self.priorityRobot = None
        self.redRobotArray = RedRobotMap()
        self.greenRobotArray = GreenRobotMap()
        self.timeStamp = 0.0

        # Create subscribers
        self.redRobotSub = rospy.Subscriber("/redbird/simulation/robots/red", RedRobotMap, self.updateRedRobotSub)
        self.greenRobotSub = rospy.Subscriber("/redbird/simulation/robots/green", GreenRobotMap, self.updateGreenRobotSub)

    def updateRedRobotSub(self, msg):
        self.redRobotArray = msg.robots

    def updateGreenRobotSub(self, msg):
        self.greenRobotArray = msg.robots

    def getRobotArray(self):
        return self.redRobotArray + self.greenRobotArray

    def getPriorityRobot(self):
        tempConfidence = -1.0
        tempPriority = None
        for robot in self.getRobotArray:
            if robot.confidence > tempConfidence and robot.out_of_bounds == False:
                tempConfidence = robot.confidence
                tempPriority = robot
        return tempPriority

    def flight(self):
        # Number of ground robots the drone will follow before landing
        rotations = 3

        # Takeoff
        self.loginfo("You can do this Corvus 1, we believe in you!")
        self.loginfo("Taking off to 2.5 meters!")
        self.takeoff_opt(2.5)

        # Hold
        self.loginfo("Altitude goal met! Holding for 5 seconds...")
        self.sleep(5.0)

        # Fly to center
        self.loginfo("Flying to center.")
        self.fly_to_point((10.0, 10.0, 2.5))

        # Flight Logic
        while rotations > 0:
            while len(self.getRobotArray) == 0:
                # Fly to center and wait for ground robotss
                self.fly_to_point((10.0, 10.0, 2.5))
            self.timeStamp = rospy.get_time()
            self.priorityRobot = self.getPriorityRobot()
            while rospy.get_time() - self.timeStamp < 120.0 and self.priorityRobot.out_of_bounds == False and len(self.getRobotArray) > 0:
                self.fly_to_point((self.vehicle.get_position_x(), self.vehicle.get_position_y(), 2.5))
            # Hold
            self.loginfo("Done following ground robot. Holding for 5 seconds...")
            self.sleep(5.0)
            # Fly to center
            self.fly_to_point((10.0, 10.0, 2.5))
            rotations-=1


        rospy.loginfo("Done. Landing.")
        self.land()