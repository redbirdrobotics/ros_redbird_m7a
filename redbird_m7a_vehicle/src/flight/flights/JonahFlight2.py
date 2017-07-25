#!/usr/bin/python

"""JonahFlight2.py: Flying around in a square."""

import rospy
import time
import flightsys
import random
from flightsys import Controller, Control_Mode

__author__ = "Jonah Largen"

random.seed()


class JonahFlight2(flightsys.Flight, object):
    NAME = 'jonah_flight2'
    LOG_TAG = 'jf2'

    def __init__(self):
        # Call super constructor
        super(JonahFlight2, self).__init__(name=self.NAME, log_tag=self.LOG_TAG)

    def flight(self):

        # Takeoff
        self.loginfo("Taking off!")
        self.takeoff_opt(2.0)

        # Hold
        self.loginfo("Finished taking off. Holding for 5 seconds...")
        self.sleep(5.0)

        # Vars
        x = self.vehicle.get_position_x()
        y = self.vehicle.get_position_y()
        z = 2.0
        rotations = 2
        offset = 1.5

        while rotations > 0:

            # Fly to point [+x]
            self.fly_to_point((x + offset, y, z))

            # Hold
            self.loginfo("Moved [+x]! Holding for 2 seconds...")
            self.sleep(2.0)

            # Fly to point [+x, +y]
            self.fly_to_point((x + offset, y + offset, z))

            # Hold
            self.loginfo("Moved [+x, +y]! Holding for 2 seconds...")
            self.sleep(2.0)

            # Fly to point [+y]
            self.fly_to_point((x, y + offset, z))

            # Hold
            self.loginfo("Moved [+y]! Holding for 2 seconds...")
            self.sleep(2.0)

            # Fly to point [origin]
            self.fly_to_point((x, y, z))

            # Hold
            self.loginfo("Moved [origin]! Holding for 2 seconds...")
            self.sleep(2.0)

            # Finish Loop
            rotations-=1
            self.loginfo("Remaining Rotations: " + str(rotations))

        # Land
        self.loginfo("Complete! Landing...")
        self.land()
