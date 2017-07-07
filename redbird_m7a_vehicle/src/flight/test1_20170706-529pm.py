#!/usr/bin/python

import rospy
import time
from utils import Vehicle, Flight_Controller, Flight_Mode
from redbird_m7a_msgs.msg import Map, GroundRobotPosition

class Flight(object):
    def __init__(self):
        # Initialize node
        rospy.init_node('test_flight_node', anonymous=True)

        # Initialize vehicle for tracking
        self._v = Vehicle()

        # Initialize flight controller
        self._fc = Flight_Controller(self._vehicle)

    def start(self):
        while not rospy.is_shutdown():
            # Arm the vehicle
            self._v.arm()

            rospy.loginfo("Position: (%0.2f, %0.2f, %0.2f)" % self._vehicle.get_position())

            # Set flight controller target velocity
            self._fc.set_velocity()

if __name__ == '__main__':
    try:
        # Initialize node
        node = Flight()

        # Start node
        node.start()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("Flight node shutdown")
        pass
