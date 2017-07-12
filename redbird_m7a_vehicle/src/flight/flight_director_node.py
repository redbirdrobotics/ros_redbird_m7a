#!/usr/bin/python

"""flight_director_node.py: The outward facing node that is launched by ROS."""

import rospy
import mavros
import flightsys
from Test_Flight import Test_Flight
from Test_Flight2 import Test_Flight2

__author__ = "Alex Bennett"
__email__ = "alex.eugene.bennett@gmail.com"


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('flight_director', disable_signals=True)

    try:
        # Initialize vehicle
        vehicle = flightsys.Vehicle()

        # Initialize flight controller
        controller = flightsys.Controller(vehicle)

        # Initialize flight director
        fd = flightsys.Flight_Director(vehicle, controller)

        # Create and add flights
        for klass in flightsys.Flight.__subclasses__():
            inst = klass(vehicle, controller)
            fd.add_flight(inst.get_name(), inst)

        # Waiting for command
        rospy.loginfo(fd.get_log_tag() + "Waiting for service call...")

        # Spin until killed
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
