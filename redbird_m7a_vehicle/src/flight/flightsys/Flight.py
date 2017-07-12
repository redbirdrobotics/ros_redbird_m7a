#!/usr/bin/python

"""Flight.py: This is a parent class from which all flights inherit and are built from."""

import rospy
from Vehicle import Vehicle
from Controller import Controller, Control_Mode

__author__ = "Alex Bennett"
__email__ = "alex.eugene.bennett@gmail.com"


class Flight(object):
    def __init__(self, name, log_tag, vehicle, controller):
        # Check parameter data types
        if not isinstance(vehicle, Vehicle) \
            or not isinstance(controller, Controller) \
            or type(name) is not str \
            or type(log_tag) is not str:
            raise TypeError("Invalid parameter data type(s)")

        # Set parameters
        self._name = name
        self._log_tag = log_tag
        self._v = vehicle
        self._c = controller

    def get_name(self):
        """Returns the flight's name."""
        return self._name

    def get_log_tag(self):
        """Returns the tag to use in the log."""
        return "[" + self._log_tag + "] "

    def start(self):
        """Runs the flight inside of a try-except block."""
        try:
            self.flight()
        except Exception:
            rospy.logerr("Flight was killed")

    def flight(self):
        """Template for function that starts flight."""
        # Raise error to highlight lack of implementation
        raise NotImplementedError("The start method has not been properly overridden by the flight implementation!")

    def kill(self):
        """Terminates the running controller thread and kills the flight."""
        self._c.set_thread_event()

