#!/usr/bin/python

import rospy
from redbird_m7a.msg

class Flight(object):
    def __init__(self):
        # Initialize node
        rospy.init_node('flight_node', anonymous=True)

    def start(self):
        while not rospy.is_shutdown():
            pass

if __name__ == '__main__':
    try:
        # Initialize node
        node = Flight()

        # Start node
        node.start()
    except rospy.ROSInterruptException:
        pass
