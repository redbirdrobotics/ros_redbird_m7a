#!/usr/bin/python

# TODO: Add header docstring

import rospy
import time
import flightsys
from flightsys import Controller, Control_Mode
from redbird_m7a_msgs.msg import Map, GroundRobotPosition


class Test_Flight(flightsys.Flight, object):
    def __init__(self, vehicle):
        # Call super constructor
        super(Test_Flight, self).__init__(name='test_flight', log_tag='[TEST FLIGHT]', vehicle=vehicle)

    def start(self):
        try:
            # Arm the vehicle
            self._v.arm()

            rospy.loginfo(self._log_tag + "Vehicle armed")

            # Wait a moment
            rospy.sleep(2)

            # Set takeoff altitude
            self._c.set_takeoff_altitude(4.0)

            print self._c.get_takeoff_altitude()

            # Switch mode to takeoff
            self._c.set_mode(Control_Mode.TAKEOFF)

            print self._c.get_mode()

            # Wait for takeoff to complete
            while not rospy.is_shutdown() and self._c.get_mode() == Control_Mode.TAKEOFF:
                pass

            # Hold
            rospy.loginfo(self._log_tag + "Altitude met, holding for 5.0 seconds")
            rospy.sleep(5)

            # Fly to point
            target_point = (5.0, 5.0, 5.0)
            rospy.loginfo(self._log_tag + "Flying to %s" % (target_point,))
            self._c.set_position(target_point)
            self._c.set_mode(Control_Mode.POSITION)

            # Wait for position to be reached
            while not rospy.is_shutdown() and self._c.get_mode() == Control_Mode.POSITION:
                pass

            # Fly up at 5 m/s for 3 seconds
            rospy.loginfo(self._log_tag + "Flying up at 5.0 m/s for 3.0 seconds")
            self._c.set_velocity((0.0, 0.0, 5.0), 3.0)
            self._c.set_mode(Control_Mode.VELOCITY)

            # Wait for velocity target to complete
            while not rospy.is_shutdown() and self._c.get_mode() == Control_Mode.VELOCITY:
                pass

            # Switch mode to land
            rospy.loginfo(self._log_tag + "Landing")
            self._c.set_mode(Control_Mode.LAND)

            # Wait for land to complete
            while not rospy.is_shutdown() and self._c.get_mode() == Control_Mode.LAND:
                pass

            # Disarm
            rospy.loginfo(self._log_tag + "Disarming")
            self._v.disarm()
        except KeyboardInterrupt:
            pass

