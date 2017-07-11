#!/usr/bin/python

# TODO: Add header docstring

import rospy
import threading
import mavros
import flightsys
import multiprocessing
from flightsys import Control_Mode
from std_msgs.msg import Empty
from redbird_m7a_msgs.srv import *
from Test_Flight import Test_Flight
from Test_Flight2 import Test_Flight2

class Flight_Director(object):
    def __init__(self, vehicle):
        # Store parameters
        self._vehicle = vehicle
        self._log_tag = "[FD] "
        self._current_flight = None

        # Flight threading
        self._flight_process = None
        self._flight_thread = None

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

        # Log shutdown complete
        rospy.loginfo(self._log_tag + "Shutdown complete")

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
        if type(name) is not str or not isinstance(flight, flightsys.Flight):
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
        # Find matching flight
        for _name, _flight in self._available_flights:
            if _name == name:
                # Log the start
                rospy.loginfo(self._log_tag + "Starting flight \"%s\"..." % name)

                # Handle multiprocessing
                try:
                    # Create process
                    self._flight_process = multiprocessing.Process(target=_flight.start, args=())

                    # Start the process
                    self._flight_process.start()

                    # Display the process identifier
                    rospy.loginfo(self._log_tag + "Flight started with pid %d" % self._flight_process.pid)

                    # # Set the current flight
                    # self._current_flight = _flight

                    # # Set flight to running
                    # self._current_flight.set_running(True)

                    # # Create the thread
                    # self._flight_thread = threading.Thread(target=_flight.start, args=())

                    # # Spawn it
                    # self._flight_thread.start()

                    # Allow thread to start
                    rospy.sleep(1)
                except Exception:
                    # Error starting thread
                    rospy.logerr("Unable to start flight thread")

                # Return after flight is complete
                return

        # If no matching flight is found, log error
        rospy.logerr("No flight found matching the name \"%s\"" % name)

    def start_flight_service_handler(self, req):
        """Handles the start_flight service request."""
        # Start the flight
        self.start_flight(req.name)

        # Return response
        return StartFlightResponse(Empty())

    def get_flights_service_handler(self, req):
        """Handles the get_flights service request."""
        # Build array of flight names
        flights = [flight[0] for flight in self._available_flights]

        # Return flights
        return GetFlightsResponse(flights)

    def kill_flight_service_handler(self, req):
        """Handles the kill_flight service request."""
        # Log event
        rospy.logwarn(self._log_tag + "Killing current flight!")

        # If kill boolean is true, kill flight process
        if req.kill and self._flight_process.is_alive():
            self._flight_process.terminate()
            self._flight_process.join()

        # Log event
        rospy.logwarn(self._log_tag + "Flight killed")

        # Return response
        return KillFlightResponse(Empty())

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('flight_director', disable_signals=True)

    try:
        # Initialize vehicle
        vehicle = flightsys.Vehicle()

        # Initialize flight director
        fd = Flight_Director(vehicle)

        # Add flights
        fd.add_flight('test_flight', Test_Flight(vehicle))
        fd.add_flight('test_flight2', Test_Flight2(vehicle))

        # Waiting for command
        rospy.loginfo(fd.get_log_tag() + "Waiting for command...")

        # Spin until killed
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
