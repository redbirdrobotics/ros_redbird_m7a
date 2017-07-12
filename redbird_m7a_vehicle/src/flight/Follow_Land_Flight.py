#!/usr/bin/python

"""Follow_Land_Flight.py: Testing of following and landing on an inline simulated ground robot."""

import rospy
import time
import flightsys
import random
from flightsys import Controller, Control_Mode
from redbird_m7a_msgs.msg import Map, GroundRobotPosition

__author__ = "Jonah Largen, Alex Bennett"


class Follow_Land_Flight(flightsys.Flight, object):
    def __init__(self, vehicle, controller):
        # Call super constructor
        super(Follow_Land_Flight, self).__init__(name='follow_land_flight', log_tag='FOLLOW/LAND', vehicle=vehicle, controller=controller)

        #vars
        self.x = 0.0
        self.y = 0.0

    def flight(self):
        # Arm the vehicle
        self.vehicle.arm()

        # Wait a moment
        self.sleep(2)

        # Set takeoff altitude
        self.takeoff(2.5)

        # Hold
        self.loginfo("Altitude met, holding for 1.0 seconds")
        self.sleep(1)

        i = 0
        while self.is_running() and (i < 5):
            target_point = (self.x, self.y, 2.5)

            self.fly_to_point(target_point)

            i+=1

        # landing logic
        alt = 2.5
        counter = 1
        while self.is_running() and (alt >= .5):
            alt-=.25
            self.x = self.x + random.uniform(-.5, .5)
            self.y = self.y + random.uniform(-.5, .5)

            target_point = (self.x, self.y, alt)

            self.fly_to_point(target_point)

        # Switch mode to land
        self.land()

        # Disarm
        self.vehicle.disarm()

