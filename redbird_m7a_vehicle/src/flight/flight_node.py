#!/usr/bin/python

import rospy
from redbird_m7a.msg
from utils import Vehicle

class Flight(object):
    def __init__(self):
        # Initialize node
        rospy.init_node('flight_node', anonymous=True)

        # Initialize vehicle for tracking
        self._vehicle = Vehicle()

    def start(self):
        while not rospy.is_shutdown():
            print self._vehicle.get_velocity()

            time.sleep(1)

if __name__ == '__main__':
    try:
        # Initialize node
        node = Flight()

        # Start node
        node.start()
    except rospy.ROSInterruptException:
        pass
