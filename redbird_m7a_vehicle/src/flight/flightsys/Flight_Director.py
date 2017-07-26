#!/usr/bin/python

"""Flight_Director.py: The flight director handles the instantiation of all flights and services."""

import rospy
import threading
import flightsys
import copy
from Queue import Queue
from std_msgs.msg import Empty, Header
from redbird_m7a_msgs.msg import FlightState, FlightInformation
from redbird_m7a_msgs.srv import *

__author__ = "Alex Bennett"


class Flight_Director(object):
    def __init__(self):
        # Create vehicle interface
        self._vehicle = flightsys.Vehicle()

        # Store parameters
        self.log_tag = "[FD] "
        self._queue_size = 1

        # Flight information
        self._current_flight = None
        self._previous_flight = None

        # Create rospy rate
        self._rate = rospy.Rate(10)

        # Log startup
        rospy.loginfo(self.log_tag + "Starting flight director...")

        # Store flights
        self._available_flights = []

        # Setup services
        self._start_flight_srv = rospy.Service('/redbird/flight_director/start_flight', StartFlight, self.start_flight_service_handler)
        self._get_flights_srv = rospy.Service('/redbird/flight_director/get_flights', GetFlights, self.get_flights_service_handler)
        self._kill_flight_srv = rospy.Service('/redbird/flight_director/kill_flight', KillFlight, self.kill_flight_service_handler)

        # Setup publishers
        self._flight_state_pub = rospy.Publisher('/redbird/flight_director/flight_state', FlightState, queue_size=self._queue_size)

        # Spawn flight state thread
        try:
            # Create thread object
            self._flight_state_thread = threading.Thread(target=self.flight_state_thread, args=())

            # Start thread
            self._flight_state_thread.start()

            # Allow thread to start
            rospy.sleep(1)
        except Exception as e:
            rospy.logerr(self.log_tag + "Unable to start flight state monitor thread: %s" % e)

        # Log service ready
        rospy.loginfo(self.log_tag + "Flight services ready")

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)

    def flight_state_thread(self):
        """Continually publishes the current flight state."""
        while not rospy.is_shutdown():
            try:
                # Create message
                msg = FlightState(header = Header(stamp=rospy.get_rostime()))
                current_flight_info = FlightInformation()
                previous_flight_info = FlightInformation()

                # Current flight information
                if self._current_flight is not None:
                    current_flight_info.name = self._current_flight.name
                    current_flight_info.start_time = self._current_flight.start_time
                    current_flight_info.end_time = self._current_flight.end_time
                    current_flight_info.end_reason = str(self._current_flight.end_reason)

                    # Add to message
                    msg.current_flight = current_flight_info

                # Previous flight information
                if self._previous_flight is not None:
                    previous_flight_info.name = self._previous_flight.name
                    previous_flight_info.start_time = self._previous_flight.start_time
                    previous_flight_info.end_time = self._previous_flight.end_time
                    previous_flight_info.end_reason = str(self._previous_flight.end_reason)

                    # Add to message
                    msg.previous_flight = previous_flight_info

                # Update is_flying tracker
                if self._current_flight is not None and not self._current_flight.shutdown_flag.is_set() and self._current_flight.start_time > 0:
                    msg.is_flying = True
                else:
                    msg.is_flying = False

                # Publish
                if not rospy.is_shutdown():
                    self._flight_state_pub.publish(msg)

                # Sleep to keep desired rate
                self._rate.sleep()
            except:
                pass

    def shutdown(self):
        """Performs shutdown actions to cleanly exit."""
        # Log shutdown
        rospy.loginfo(self.log_tag + "Shutting down flight director...")

        # Kill flight if one exists
        if self._current_flight is not None:
            self._current_flight.shutdown_flag.set()

        # Disarm
        self._vehicle.disarm()

        # Log shutdown complete
        rospy.loginfo(self.log_tag + "Shutdown complete")

    def get_kill_flight_flag(self):
        """Returns the kill flight threading event."""
        return self._kill_flight

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
        rospy.loginfo(self.log_tag + "Flight \"%s\" loaded" % name)

    def get_flights(self):
        """Returns a list of all available flights."""
        return self._available_flights

    def start_flight(self, name):
        """Starts the flight with a name matching the name provided.

        Args:
            name (string): The name/identifier of the flight to start.
        """
        # Log the attempt
        rospy.loginfo(self.log_tag + "Attempting to start flight...")

        # First verify that no flight is currently running
        if self._current_flight is not None and self._current_flight.thread.is_alive():
            rospy.logerr(self.log_tag + "\"%s\" is currently running!" % self._current_flight.name)

            # Error, return false
            return False

        # Find matching flight
        for _name, _flight in self._available_flights:
            if _name == name:
                # Store to previous flight
                self._previous_flight = copy.copy(self._current_flight)

                # Store current flight
                self._current_flight = _flight

                # Create the thread
                self._current_flight.thread = threading.Thread(target=self._current_flight.run)

                # Reset event
                self._current_flight.shutdown_flag.clear()

                # Set to daemon
                self._current_flight.thread.daemon = True

                # Start the flight
                self._current_flight.thread.start()

                # Display the process identifier
                rospy.loginfo(self.log_tag + "\"%s\" flight thread started (id: %s)" % (_name, self._current_flight.thread.ident))

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
        if req.kill:
            # Check that flight is running
            if self._current_flight is None or not self._current_flight.thread.is_alive():
                rospy.logwarn("No flight to kill")
            elif self._current_flight is not None and self._current_flight.thread.is_alive():
                # Log event
                rospy.logwarn(self.log_tag + "Killing current flight!")

                # Disarm
                self._current_flight.vehicle.disarm()

                # Set poison pill to kill thread
                self._current_flight.shutdown_flag.set()

                # Set the end state
                self._current_flight.end_reason = flightsys.Flight_End_Reason.KILLED

                # Join and wait for termination
                self._current_flight.thread.join()

                # Response flag
                success = True

                # Log event
                rospy.logwarn(self.log_tag + "Flight killed")
        else:
            # Log error
            rospy.logerr(self.log_tag + "Flight not killed")

        # Return response
        return KillFlightResponse(success=success)
