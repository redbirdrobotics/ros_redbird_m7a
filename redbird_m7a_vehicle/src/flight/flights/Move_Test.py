#!/usr/bin/python

"""Move_Test.py: A quick demonstration of takeoff and landing capability."""

import rospy
import time
import flightsys
from flightsys import Controller, Control_Mode

__author__ = "Alex Bennett"


class Move_Test(flightsys.Flight, object):
    NAME = 'move_test'
    LOG_TAG = 'mt'

    def __init__(self):
        # Call super constructor
        super(Move_Test, self).__init__(name=self.NAME, log_tag=self.LOG_TAG)

    def flight(self):
        # Takeoff
        self.loginfo("Taking off!")
        self.takeoff_opt(2.0)

        # Hold
        self.loginfo("Altitude goal met! Holding for 5 seconds...")
        self.sleep(5.0)

        # Move
        move_x = self.vehicle.get_position_x() + 5.0
        move_y = self.vehicle.get_position_y() + 0.0
        self.fly_to_point((move_x, move_y, 2.0))

        # Hold
        self.loginfo("Reached new position! Holding for 5 seconds...")
        self.sleep(5.0)

        # Move
        move_x = self.vehicle.get_position_x() - 5.0
        move_y = self.vehicle.get_position_y() + 0.0
        self.fly_to_point((move_x, move_y, 1.0))

        # Hold
        self.loginfo("Reached new position! Holding for 5 seconds...")
        self.sleep(5.0)

        # Land
        self.loginfo("Landing...")
        self.land()

        #Done
        self.loginfo("Done!")
