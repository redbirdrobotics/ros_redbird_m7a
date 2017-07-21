#!/usr/bin/python

"""Takeoff_Land.py: A quick demonstration of takeoff and landing capability."""

import rospy
import time
import flightsys
from flightsys import Controller, Control_Mode

__author__ = "Alex Bennett"


class Takeoff_Land(flightsys.Flight, object):
    def __init__(self, vehicle):
        # Call super constructor
        super(Takeoff_Land, self).__init__(name='takeoff_land_flight', log_tag='TEST #1', vehicle=vehicle)

    def flight(self):
        # Takeoff
        self.takeoff(1.5)

        # Hold
        self.loginfo("Altitude goal met! Holding for 5 seconds...")
        self.sleep(2.0)

        # Land
        self.land()
