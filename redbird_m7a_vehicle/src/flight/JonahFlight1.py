#!/usr/bin/python

"""Test_Flight.py: A quick demonstration of the capabilities of the flight system."""

import rospy
import time
import flightsys
import random
from flightsys import Controller, Control_Mode

__author__ = "Jonah Largen"

random.seed()


class JonahFlight1(flightsys.Flight, object):
    NAME = 'jonah_flight1'
    LOG_TAG = 'jf1'

    def __init__(self):
        # Call super constructor
        super(JonahFlight1, self).__init__(name=self.NAME, log_tag=self.LOG_TAG)

    def flight(self):
        x = self.vehicle.get_position_x()
        y = self.vehicle.get_position_y()
        z = 2.5
        rotations = 5;

        # Takeoff
        self.loginfo("Taking off to %s" % str(z))
        self.takeoff(z)

        # Fly to point
        self.fly_to_point((x, y, z), 0.2)

        # Hold
        self.loginfo("Goal met at center! Holding for 2 seconds...")
        self.sleep(2.0)

        #Generate random position [pretend robot position in future]
        x = x + random.uniform(-3.0, 3.0)
        y = y + random.uniform(-3.0, 3.0)
        self.loginfo("Coordinates = X: " + str(x) + ", Y: + " + str(y) + ", Z: + " + str(z))

        # Fly to point
        self.fly_to_point((x, y, z), 0.2)

        # Hold
        self.loginfo("Goal met! Holding for 3 seconds...")
        self.sleep(3.0)

        # Fly to point [ground]
        z = 1.0
        self.fly_to_point((x, y, z), 0.2)

        # Rotate robot
        while (rotations > 0):
            self.loginfo("Rotations remaining: %s" % rotations)
            self.loginfo("Going up")
            self.loginfo("Coordinates = X: " + str(x) + ", Y: + " + str(y) + ", Z: + " + str(z))
            self.fly_to_point((x, y, z + 0.2), 0.05)

            self.loginfo("Goal met! Holding for 0.5 seconds...")
            self.sleep(0.5)

            self.loginfo("Going down")
            self.loginfo("Coordinates = X: " + str(x) + ", Y: + " + str(y) + ", Z: + " + str(z))
            self.fly_to_point((x, y, z), 0.05)

            self.loginfo("Goal met! Holding for 0.5 seconds...")
            self.sleep(0.5)

            rotations-=1

        # Fly to point [sky]
        x = self.vehicle.get_position_x()
        y = self.vehicle.get_position_y()
        z = 2.5
        self.fly_to_point((x, y, z), 0.2)

        #Fly to point [downward]
        self.fly_to_point((x, y, z-2.0), 0.2)

        # Land
        self.land()
