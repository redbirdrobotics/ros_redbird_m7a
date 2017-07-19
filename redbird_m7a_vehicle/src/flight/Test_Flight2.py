#!/usr/bin/python

# TODO: Add header docstring

import rospy
import time
import flightsys
import random
from flightsys import Controller, Control_Mode
from redbird_m7a_msgs.msg import Map, GroundRobotPosition


class Test_Flight2(flightsys.Flight, object):
    def __init__(self, vehicle):
        # Call super constructor
        super(Test_Flight2, self).__init__(name='test_flight2', log_tag='[TEST FLIGHT 2]', vehicle=vehicle)
        
        #vars
        this.x = 0
        this.y = 0

    def start(self):
        try:
            # Arm the vehicle
            self._v.arm()

            rospy.loginfo(self._log_tag + "Vehicle armed")

            # Wait a moment
            rospy.sleep(2)

            # Set takeoff altitude
            self._c.set_takeoff_altitude(2.5)

            print self._c.get_takeoff_altitude()

            # Switch mode to takeoff
            self._c.set_mode(Control_Mode.TAKEOFF)

            print self._c.get_mode()

            # Wait for takeoff to complete
            while not rospy.is_shutdown() and self._c.get_mode() == Control_Mode.TAKEOFF:
                pass

            # Hold
            rospy.loginfo(self._log_tag + "Altitude met, holding for 1.0 seconds")
            rospy.sleep(1)
            
            x = 0
            while (x < 5):
                target_point = (self.x, self.y, 2.5)
                rospy.loginfo(self._log_tag + "Flying to %s" % (target_point,))
                self._c.set_position(target_point)
                self._c.set_mode(Control_Mode.POSITION)

                # Wait for position to be reached
                while not rospy.is_shutdown() and self._c.get_mode() == Control_Mode.POSITION:
                    pass
                
                x+=1
            
            # landing logic
            alt = 2.5
            counter = 1
            while (alt >= .2):
                alt-=.1
                x = x + random.uniform(-.5, .5)
                y = y + random.uniform(-.5, .5)
                target_point = (self.x, self.y, alt)
                rospy.loginfo(self._log_tag + "Flying to %s" % (target_point,))
                self._c.set_position(target_point)
                self._c.set_mode(Control_Mode.POSITION)

                # Wait for position to be reached
                while not rospy.is_shutdown() and self._c.get_mode() == Control_Mode.POSITION:
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

