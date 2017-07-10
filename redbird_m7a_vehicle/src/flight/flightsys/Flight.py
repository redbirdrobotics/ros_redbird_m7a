#!/usr/bin/python

# TODO: Add header docstring

import rospy
from Vehicle import Vehicle
from Controller import Controller, Control_Mode


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
        self._log_tag = log_tag + " "
        self._v = vehicle
        self._c = controller

    def get_name(self):
        """Returns the flight's name."""
        return self._name

    def get_log_tag(self):
        """Returns the tag to use in the log."""
        return self._log_tag

    def start(self):
        """Template for function that starts flight."""
        # Raise error to highlight lack of implementation
        raise NotImplementedError("The start method has not been properly overridden by the flight implementation!")

    def shutdown(self):
        """Performs shutdown actions to cleanly exit."""

        # Log shutdown
        rospy.loginfo(self._log_tag + "Shutting down...")

        # Disarm
        self._v.disarm()

        # Reset Flight_Mode
        self._c.set_mode(Control_Mode.INITIAL)

        # Log shutdown complete
        rospy.loginfo(self._log_tag + "Shutdown complete")

        while True:
            pass
