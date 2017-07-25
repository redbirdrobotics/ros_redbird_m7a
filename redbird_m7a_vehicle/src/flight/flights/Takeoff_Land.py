#!/usr/bin/python

"""Takeoff_Land.py: A quick demonstration of takeoff and landing capability."""

import rospy
import time
import flightsys
from flightsys import Controller, Control_Mode

__author__ = "Alex Bennett"


class Takeoff_Land(flightsys.Flight, object):
    NAME = 'takeoff_land'
    LOG_TAG = 't&l'

    def __init__(self):
        # Call super constructor
        super(Takeoff_Land, self).__init__(name=self.NAME, log_tag=self.LOG_TAG)

    def flight(self):
        # Takeoff
        self.loginfo("Taking off!")
        self.takeoff_opt(1.5)

        # Hold
        self.loginfo("Altitude goal met! Holding for 5 seconds...")
        self.sleep(5.0)

        # Move higher
        self.loginfo("Going higher!")
        self.fly_to_point((self.vehicle.get_position_x(), self.vehicle.get_position_y(), 3.0))

        # Wait
        self.loginfo("Holding")
        self.sleep(5.0)

        # Land
        self.loginfo("Landing...")
        self.land()

        #Done
        self.loginfo("Done!")
