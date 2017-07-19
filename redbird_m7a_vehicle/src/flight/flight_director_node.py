#!/usr/bin/python

"""flight_director_node.py: Frontend ROS node implementation of the Flight_Director handler."""

import rospy
import mavros
import flightsys
from Test_Flight import Test_Flight
from Follow_Land_Flight import Follow_Land_Flight
from Takeoff_Land import Takeoff_Land

__author__ = "Alex Bennett"

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('flight_director', disable_signals=True)

    try:
        # Initialize vehicle
        vehicle = flightsys.Vehicle()

        # Initialize flight director
        fd = flightsys.Flight_Director(vehicle)

        # Create and add flights
        for klass in flightsys.Flight.__subclasses__():
            inst = klass(vehicle)
            fd.add_flight(inst.name, inst)

        # Waiting for command
        rospy.loginfo(fd.log_tag + "Waiting for service call...")

        # Spin until killed
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
