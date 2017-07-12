#!/usr/bin/python

"""Test_Flight2: Testing of following and landing on an inline simulated ground robot."""

import rospy
import time
import flightsys
import random
from flightsys import Controller, Control_Mode
from redbird_m7a_msgs.msg import Map, GroundRobotPosition

__author__ = "Jonah Largen, Alex Bennett"


class Test_Flight2(flightsys.Flight, object):
    def __init__(self, vehicle, controller):
        # Call super constructor
        super(Test_Flight2, self).__init__(name='follow_land_test', log_tag='FOLLOW & LAND', vehicle=vehicle, controller=controller)

        #vars
        self.x = 0.0
        self.y = 0.0

    def flight(self):
        # Arm the vehicle
        self._v.arm()

        # Wait a moment
        rospy.sleep(2)

        # Set takeoff altitude
        self._c.set_takeoff_altitude(2.5)

        # Switch mode to takeoff
        self._c.set_mode(Control_Mode.TAKEOFF)

        # Wait for takeoff to complete
        self._c.wait_for_mode_change(Control_Mode.TAKEOFF)

        # Hold
        rospy.loginfo(self._log_tag + "Altitude met, holding for 1.0 seconds")
        rospy.sleep(1)

        i = 0
        while not rospy.is_shutdown() and (i < 5):
            target_point = (self.x, self.y, 2.5)
            self._c.set_position(target_point)
            self._c.set_mode(Control_Mode.POSITION)

            # Wait for position to be reached
            self._c.wait_for_mode_change(Control_Mode.POSITION)

            i+=1

        # landing logic
        alt = 2.5
        counter = 1
        while not rospy.is_shutdown() and (alt >= .5):
            alt-=.25
            self.x = self.x + random.uniform(-.5, .5)
            self.y = self.y + random.uniform(-.5, .5)

            target_point = (self.x, self.y, alt)

            self._c.set_position(target_point)
            self._c.set_mode(Control_Mode.POSITION)

            # Wait for position to be reached
            self._c.wait_for_mode_change(Control_Mode.POSITION)

        # Switch mode to land
        rospy.loginfo(self._log_tag + "Landing")
        self._c.set_mode(Control_Mode.LAND)

        # Wait for land to complete
        self._c.wait_for_mode_change(Control_Mode.LAND)

        # Disarm
        rospy.loginfo(self._log_tag + "Disarming")
        self._v.disarm()

