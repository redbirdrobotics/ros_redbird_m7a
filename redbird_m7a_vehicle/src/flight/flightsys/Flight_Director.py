#!/usr/bin/python

"""Flight_Director.py: The flight director handles the instantiation of all flights and services."""

import rospy
import threading
from std_msgs.msg import Empty
from redbird_m7a_msgs.srv import *
from Flight import Flight

__author__ = "Alex Bennett"
__email__ = "alex.eugene.bennett@gmail.com"


class Flight_Director(object):
    def __init__(self, vehicle, controller):
        # Store parameters
        self._vehicle = vehicle
        self._controller = controller
        self._log_tag = "[FD] "
        self._current_flight = None

        # Flight threading
        self._kill_flight = threading.Event()
        self._flight_process = None
        self._flight_thread = threading.Thread()

        # Log startup
        rospy.loginfo(self._log_tag + "Starting flight director...")

        # Store flights
        self._available_flights = []

        # Setup services
        self._start_flight_srv = rospy.Service('start_flight', StartFlight, self.start_flight_service_handler)
        self._get_flights_srv = rospy.Service('get_flights', GetFlights, self.get_flights_service_handler)
        self._kill_flight_srv = rospy.Service('kill_flight', KillFlight, self.kill_flight_service_handler)

        # Log service ready
        rospy.loginfo(self._log_tag + "Flight services ready")

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        """Performs shutdown actions to cleanly exit."""
        # Log shutdown
        rospy.loginfo(self._log_tag + "Shutting down current flight...")

        # Disarm
        self._vehicle.disarm()

        # Kill the controller
        self._controller.kill()

        # Wait for the controller to die
        while self._controller.is_running():
            pass

        # Log shutdown complete
        rospy.loginfo(self._log_tag + "Shutdown complete")

    def get_kill_flight_flag(self):
        """Returns the kill flight threading event."""
        return self._kill_flight

    def get_log_tag(self):
        """Returns the log tag."""
        return self._log_tag

    def add_flight(self, name, flight):
        """Adds a flight to the list of available flights.

        Args:
            name (string): The name that will be used to identify the flight.
            flight (function): Instance of the flight object.
        """
        # Verify parameter data types
        if type(name) is not str or not isinstance(flight, Flight):
            raise TypeError('Specified parameter data type is incorrect')

        # Append to list
        self._available_flights.append((name, flight))

        # Log added flight
        rospy.loginfo(self._log_tag + "Flight \"%s\" loaded" % name)

    def get_flights(self):
        """Returns a list of all available flights."""
        return self._available_flights

    def start_flight(self, name):
        """Starts the flight with a name matching the name provided.

        Args:
            name (string): The name/identifier of the flight to start.
        """
        # First verify that no flight is currently running
        if self._flight_thread.is_alive():
            rospy.logerr("There is a flight currently running! (name: \"%s\")" % self._flight_thread.name)

            # Error, return false
            return False

        # Find matching flight
        for _name, _flight in self._available_flights:
            if _name == name:
                # Log the start
                rospy.loginfo(self._log_tag + "Starting flight \"%s\"..." % name)

                # Handle multiprocessing
                try:
                    # Create process
                    self._flight_thread = threading.Thread(target=_flight.start, args=())

                    # Name the thread
                    self._flight_thread.name = name

                    # Clear kill flight flag
                    self._kill_flight.clear()

                    # Start the thread
                    self._flight_thread.start()

                    # Display the process identifier
                    rospy.loginfo(self._log_tag + "Flight started with identifier %d" % self._flight_thread.ident)

                    rospy.sleep(1)
                except Exception:
                    # Error starting thread
                    rospy.logerr("Could not start flight thread")

                # Return after flight is complete
                return True

        # If no matching flight is found, log error
        rospy.logerr("No flight found matching the name \"%s\"" % name)

        # Error, return false
        return False

    def start_flight_service_handler(self, req):
        """Handles the start_flight service request."""
        # Try to start the flight
        success = self.start_flight(req.name)

        # Return response
        return StartFlightResponse(success=success)

    def get_flights_service_handler(self, req):
        """Handles the get_flights service request."""
        # Build array of flight names
        flights = [flight[0] for flight in self._available_flights]

        # Return flights
        return GetFlightsResponse(flights)

    def kill_flight_service_handler(self, req):
        """Handles the kill_flight service request."""
        # Response flag
        success = False

        # If kill boolean is true
        if req.kill and self._flight_thread.is_alive():
            # Log event
            rospy.logwarn(self._log_tag + "Killing current flight!")

            # Set poison pill
            self._kill_flight.set()

            # Reset controller
            self._controller.reset()

            # Response flag
            success = True

            # Log event
            rospy.logwarn(self._log_tag + "Flight killed")

        # Return response
        return KillFlightResponse(success=success)
