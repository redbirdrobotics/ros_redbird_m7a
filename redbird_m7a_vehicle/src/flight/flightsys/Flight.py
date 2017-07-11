#!/usr/bin/python

# TODO: Add header docstring

import rospy
from Vehicle import Vehicle
from Controller import Controller, Control_Mode


class Flight(object):
    def __init__(self, name, log_tag, vehicle):
        # Check parameter data types
        if not isinstance(vehicle, Vehicle) \
            or type(name) is not str \
            or type(log_tag) is not str:
            raise TypeError("Invalid parameter data type(s)")

        # Set parameters
        self._name = name
        self._log_tag = log_tag + " "
        self._v = vehicle
        self._c = Controller(self._v)
        self._is_running = False

    def get_name(self):
        """Returns the flight's name."""
        return self._name

    def get_log_tag(self):
        """Returns the tag to use in the log."""
        return self._log_tag

    def is_running(self):
        """Returns the current running status of the flight."""
        return self._is_running

    def set_running(self, status):
        """Sets the running state of the flight.

        Args:
            running (bool): True if flight is allowed to run.
        """
        # Verify parameter data type
        if type(status) is not bool:
            raise TypeError("Invalid parameter data type(s)")

        # Set running status
        self._is_running = status

    def start(self):
        """Template for function that starts flight."""
        # Raise error to highlight lack of implementation
        raise NotImplementedError("The start method has not been properly overridden by the flight implementation!")

    def kill(self):
        """Terminates the running controller thread and kills the flight."""
        self._c.set_thread_event()
