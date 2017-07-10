#!/usr/bin/python

# TODO: Add header docstring

import rospy
import mavros
import flightsys
from Test_Flight import Test_Flight

class Flight_Director(object):
    def __init__(self):
        # Store flights
        self._available_flights = []

    def add_flight(self, name, flight):
        """Adds a flight to the list of available flights.

        Args:
            name (string): The name that will be used to identify the flight.
            flight (function): Instance of the flight object.
        """
        # Verify parameter data types
        if type(name) is not str or not isinstance(flight, flightsys.Flight):
            raise TypeError('Specified parameter data type is incorrect')

        # Append to list
        self._available_flights.append((name, flight))

    def get_flights(self):
        """Returns a list of all available flights."""
        return self._available_flights

    def start_flight(self, name):
        # Find matching flight
        for n, f in self._available_flights:
            if n == name:
                # Log the start
                rospy.loginfo("Starting flight \"%s\"..." % name)

                # Start the flight
                f.start()

                # Return after flight is complete
                return

        # If no matching flight is found, log error
        rospy.logerr("No flight found matching the name \"%s\"" % name)

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('flight_director')

    try:
        # Initialize flight director
        fd = Flight_Director()

        # Initialize vehicle
        vehicle = flightsys.Vehicle()

        # Initialize flight controller
        controller = flightsys.Controller(vehicle)

        # Add flights
        fd.add_flight('test_flight', Test_Flight(vehicle, controller))

        # Run flight
        fd.start_flight('test_flight')
    except rospy.ROSInterruptException:
        pass
