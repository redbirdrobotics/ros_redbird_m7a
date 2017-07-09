#!/usr/bin/python

# TODO: Add header docstring

import rospy
import time
from utils import Vehicle, Flight_Controller, Flight_Mode
from redbird_m7a_msgs.msg import Map, GroundRobotPosition


class Flight(object):
    def __init__(self):
        # Initialize vehicle for tracking
        self._v = Vehicle()

        # Initialize flight controller
        self._fc = Flight_Controller(self._v)

        # Log tag
        self._log_tag = "[FLIGHT] "

    def start(self):
        # Arm the vehicle
        self._v.arm()

        rospy.loginfo(self._log_tag + "Vehicle armed")

        # Wait a moment
        rospy.sleep(2)

        # Set takeoff altitude
        self._fc.set_takeoff_altitude(2.0)

        # Switch mode to takeoff
        self._fc.set_mode(Flight_Mode.TAKEOFF)

        # Wait for takeoff to complete
        while self._fc.get_mode() == Flight_Mode.TAKEOFF:
            pass

        # Hold
        rospy.loginfo(self._log_tag + "Altitude met, holding for 5.0 seconds")
        rospy.sleep(5)

        # Fly to point
        target_point = (5.0, 5.0, 5.0)
        rospy.loginfo(self._log_tag + "Flying to %s" % (target_point,))
        self._fc.set_position(target_point)
        self._fc.set_mode(Flight_Mode.POSITION)

        # Wait for position to be reached
        while self._fc.get_mode() == Flight_Mode.POSITION:
            pass

        # Fly up at 5 m/s for 3 seconds
        rospy.loginfo(self._log_tag + "Flying up at 5.0 m/s for 3.0 seconds")
        self._fc.set_velocity((0.0, 0.0, 5.0), 3.0)
        self._fc.set_mode(Flight_Mode.VELOCITY)

        # Wait for velocity target to complete
        while self._fc.get_mode() == Flight_Mode.VELOCITY:
            pass

        # Switch mode to land
        rospy.loginfo(self._log_tag + "Landing")
        self._fc.set_mode(Flight_Mode.LAND)

        # Wait for land to complete
        while self._fc.get_mode() == Flight_Mode.LAND:
            pass

        # Disarm
        rospy.loginfo(self._log_tag + "Disarming")
        self._v.disarm()

        # Spin until killed
        rospy.spin()

    def shutdown(self):
        """Performs shutdown actions to cleanly exit."""

        # Log shutdown
        rospy.loginfo(self._log_tag + "Shutting down...")

        # Disarm
        self._v.disarm()

        # Reset Flight_Mode
        self._fc.set_mode(Flight_Mode.INITIAL)

        # Log shutdown complete
        rospy.loginfo(self._log_tag + "Shutdown complete")

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('test_takeoff_node')

    try:
        # Initialize flight
        flight = Flight()

        # Register shutdown hook
        rospy.on_shutdown(flight.shutdown)

        # Start flight
        flight.start()

    except rospy.ROSInterruptException:
        rospy.loginfo("Flight node shutdown")
