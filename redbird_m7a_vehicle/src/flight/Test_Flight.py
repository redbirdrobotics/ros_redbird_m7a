#!/usr/bin/python

"""Test_Flight.py: A quick demonstration of the capabilities of the flight system."""

import rospy
import time
import flightsys
from flightsys import Controller, Control_Mode

__author__ = "Alex Bennett"


class Test_Flight(flightsys.Flight, object):
    NAME = 'test_flight'
    LOG_TAG = 'tf'

    def __init__(self):
        # Call super constructor
        super(Test_Flight, self).__init__(name=self.NAME, log_tag=self.LOG_TAG)

    def flight(self):
        # Takeoff
        self.takeoff(4.0)

        # Hold
        self.loginfo("Altitude goal met! Holding for 5 seconds...")
        self.sleep(5.0)

        # Fly to point
        self.fly_to_point((5.0, 5.0, 5.0))

        # Fly velocity
        self.fly_velocity((0.0, -6.0, 5.0), time=3.0)

        # Fly to point
        self.fly_to_point((0.0, 0.0, 2.0))

        # Land
        self.land()
