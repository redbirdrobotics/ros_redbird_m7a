#!/usr/bin/python

"""flight_director_node.py: Frontend ROS node implementation of the Flight_Director handler."""

import rospy
import mavros
import flightsys
from flights import *

__author__ = "Alex Bennett"

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('flight_director', disable_signals=True)

    try:
        # Initialize flight director
        fd = flightsys.Flight_Director()

        # Create and add flights
        for klass in flightsys.Flight.__subclasses__():
            inst = klass()
            fd.add_flight(inst.name, inst)

        # Waiting for command
        rospy.loginfo(fd.log_tag + "Waiting for service call...")

        # Spin until killed
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
