#!/usr/bin/python

"""Test_Flight.py: A quick demonstration of the capabilities of the flight system."""

import rospy
import time
import flightsys
import random
from flightsys import Controller, Control_Mode

__author__ = "Alex Bennett"
__email__ = "alex.eugene.bennett@gmail.com"


class Test_Flight(flightsys.Flight, object):
    def __init__(self, vehicle):
        # Call super constructor
        super(Test_Flight, self).__init__(name='test_flight', log_tag='TEST FLIGHT', vehicle=vehicle)

    def flight(self):
    
        x = 0.0
        y = 0.0
        z = 2.5
        rotations = 4;
        
        # Takeoff
        self.takeoff(z)

        # Hold
        self.loginfo("Altitude goal met! Holding for 2 seconds...")
        self.sleep(2.0)

        # Fly to point
        self.fly_to_point((x, y, z))
        
        #Generate random position [pretend robot position in future]
        x = random.uniform(-10.0, 10.0)
        y = random.uniform(-10.0, 10.0)
        
        # Hold
        self.loginfo("Altitude goal met! Holding for 3 seconds...")
        self.sleep(3.0)

        # Fly to point [ground]
        z = 0.5
        self.fly_to_point((x, y, z))
        
        # Rotate robot
        while (rotations > 0):
            self.fly_to_point((x, y, z + 0.25))
            self.fly_to_point((x, y, z))
            rotations-=1

        # Fly to point [sky]
        z = 2.5
        self.fly_to_point((x, y, z))

        # Land
        self.land()
