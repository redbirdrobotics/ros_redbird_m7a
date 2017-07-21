#!/usr/bin/python

"""Flight.py: This is a parent class from which all flights inherit and are built from."""

import threading
import rospy
from enum import Enum
import flightsys

__author__ = "Alex Bennett"
__email__ = "alex.eugene.bennett@gmail.com"


class Flight_End_Reason(Enum):
    INITIAL, NATURAL, KILLED = range(0, 3)


class Flight(object):
    def __init__(self, name, log_tag, vehicle):
        # Check parameter data types
        if not isinstance(vehicle, flightsys.Vehicle) \
            or type(name) is not str \
            or type(log_tag) is not str:
            raise TypeError("Invalid parameter data type(s)")

        # Create thread
        self.thread = threading.Thread()

        # Call super
        self.shutdown_flag = threading.Event()

        # Create empty controller
        self.controller = None

        # Set parameters
        self.name = name
        self.start_time = 0.0
        self.end_time = 0.0
        self.end_reason = Flight_End_Reason.INITIAL
        self.log_tag = "[" + log_tag + "] "
        self.vehicle = vehicle

    def run(self):
        """Runs the flight inside of a try-except block."""
        try:
            # Create controller
            self.controller = flightsys.Controller(self.vehicle, self.shutdown_flag)

            # Reset flight diagnositcs
            self.start_time = 0.0
            self.end_time = 0.0
            self.end_reason = Flight_End_Reason.INITIAL

            # Arm the vehicle
            self.vehicle.arm()

            # Flight starting
            rospy.loginfo(self.log_tag + "Flight ready. Waiting for OFFBOARD...")

            # Wait for offboard
            while self.vehicle.get_mode() != 'OFFBOARD' and not self.shutdown_flag.is_set() and not rospy.is_shutdown():
                pass

            # Wait a few seconds
            rospy.loginfo(self.log_tag + "OFFBOARD entered. Starting flight in 2 seconds...")
            self.sleep(2)

            # Set start time
            self.start_time = rospy.get_time()

            # Start flight
            self.flight()

            # Set end time
            self.end_time = rospy.get_time()

            # Set end type
            self.end_reason = Flight_End_Reason.NATURAL

            # Flight complete
            rospy.loginfo(self.log_tag + "Flight complete")

            # Set poison
            self.shutdown_flag.set()
        except Exception as e:
            # Set poison
            self.shutdown_flag.set()

            # Record end time
            self.end_time = rospy.get_time()

            # Set end state
            self.end_reason = Flight_End_Reason.KILLED

            # Log error
            rospy.logerr(self.log_tag + "%s" % str(e))

    def flight(self):
        """Template for function that starts flight."""
        # Raise error to highlight lack of implementation
        raise NotImplementedError("The start method has not been properly overridden by the flight implementation!")

    def is_running(self):
        return not rospy.is_shutdown() and not self.shutdown_flag.is_set() and self.vehicle.is_armed() and self.vehicle.get_mode() == 'OFFBOARD'

    def sleep(self, time):
        self.shutdown_flag.wait(time)

    def loginfo(self, msg):
        # Disallow if poison pill has been set or ROS is shutdown
        if self.shutdown_flag.is_set() or rospy.is_shutdown():
            raise Exception("Flight killed")

        # Log
        rospy.loginfo(self.log_tag + msg)

    def logwarn(self, msg):
        # Disallow if poison pill has been set or ROS is shutdown
        if self.shutdown_flag.is_set() or rospy.is_shutdown():
            raise Exception("Flight killed")

        # Log
        rospy.logwarn(self.log_tag + msg)

    def logerr(self, msg):
        # Disallow if poison pill has been set or ROS is shutdown
        if self.shutdown_flag.is_set() or rospy.is_shutdown():
            raise Exception("Flight killed")

        # Log
        rospy.logerr(self.log_tag + msg)

    def fly_to_point(self, point, allowed_error):
        # Disallow if poison pill has been set or ROS is shutdown
        if self.shutdown_flag.is_set() or rospy.is_shutdown():
            raise Exception("Flight killed")

        # Set target point
        self.controller.set_position(point, allowed_error)

        # Set mode
        self.controller.set_mode(flightsys.Control_Mode.POSITION)

        # Wait for position to be reached
        self.controller.wait_for_mode_change(flightsys.Control_Mode.POSITION)

    def fly_velocity(self, velocity, time=0.0):
        # Disallow if poison pill has been set or ROS is shutdown
        if self.shutdown_flag.is_set() or rospy.is_shutdown():
            raise Exception("Flight killed")

        # Set target velocity and time
        self.controller.set_velocity(velocity, time)

        # Set mode
        self.controller.set_mode(flightsys.Control_Mode.VELOCITY)

        # Wait for velocity target to complete
        self.controller.wait_for_mode_change(flightsys.Control_Mode.VELOCITY)

    def takeoff(self, altitude=0.0):
        # Disallow if poison pill has been set or ROS is shutdown
        if self.shutdown_flag.is_set() or rospy.is_shutdown():
            raise Exception("Flight killed")

        # Set takeoff altitude
        if altitude > 0.0:
            self.controller.set_takeoff_altitude(altitude)

        # Switch mode to takeoff
        self.controller.set_mode(flightsys.Control_Mode.TAKEOFF)

        # Wait for takeoff to complete
        self.controller.wait_for_mode_change(flightsys.Control_Mode.TAKEOFF)

    def land(self):
        # Disallow if poison pill has been set or ROS is shutdown
        if self.shutdown_flag.is_set() or rospy.is_shutdown():
            raise Exception("Flight killed")

        # Switch mode to land
        self.controller.set_mode(flightsys.Control_Mode.LAND)

        # Wait for land to complete
        self.controller.wait_for_mode_change(flightsys.Control_Mode.LAND)
