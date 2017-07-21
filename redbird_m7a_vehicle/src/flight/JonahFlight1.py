#!/usr/bin/python

"""Test_Flight.py: A quick demonstration of the capabilities of the flight system."""

import rospy
import time
import flightsys
import random
from flightsys import Controller, Control_Mode

__author__ = "Alex Bennett"
__email__ = "alex.eugene.bennett@gmail.com"

random.seed()

class JonahFlight1(flightsys.Flight, object):
    def __init__(self, vehicle):
        # Call super constructor
        super(JonahFlight1, self).__init__(name='jonah_flight1', log_tag='Jonah Flight 1', vehicle=vehicle)

    def flight(self):

        x = 0.0
        y = 0.0
        z = 2.5
        rotations = 5;

        # Takeoff
        self.takeoff(z-2.0)

        # Fly to point
        self.fly_to_point((x, y, z), 0.2)
        
        # Hold
        self.loginfo("Altitude goal met at center! Holding for 2 seconds...")
        self.sleep(2.0)
        
        #Generate random position [pretend robot position in future]
        x = random.uniform(-3.0, 3.0)
        y = random.uniform(-3.0, 3.0)
        self.loginfo("Coordinates= X: " + str(x) + " - Y: + " + str(y) + " - Z: + " + str(z))
        
        # Fly to point
        self.fly_to_point((x, y, z), 0.2)

        # Hold
        self.loginfo("Altitude goal met! Holding for 3 seconds...")
        self.sleep(3.0)

        # Fly to point [ground]
        z = 1.0
        self.fly_to_point((x, y, z), 0.2)
        
        # Rotate robot
        while (rotations > 0):
            self.fly_to_point((x, y, z + 0.2), 0.05)
            self.loginfo("Coordinates= X: " + str(x) + " - Y: + " + str(y) + " - Z: + " + str(z))
            self.loginfo("Altitude goal met! Holding for 0.5 seconds...")
            self.sleep(0.5)
            self.fly_to_point((x, y, z), 0.05)
            self.loginfo("Coordinates= X: " + str(x) + " - Y: + " + str(y) + " - Z: + " + str(z))
            self.loginfo("Altitude goal met! Holding for 0.5 seconds...")
            self.sleep(0.5)
            rotations-=1

        # Fly to point [sky]
        x = 0.0
        y = 0.0
        z = 2.5
        self.fly_to_point((x, y, z), 0.2)
        
        
        #Fly to point [downward]
        self.fly_to_point((x, y, z-2.0), 0.2)

        # Land
        self.land()
