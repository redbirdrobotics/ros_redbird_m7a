#!/usr/bin/python

import rospy
from redbird_m7a.msg import Map, GroundRobotPosition

class Localization_Publisher(object):
    def __init__(self):
        # Create publisher
        self._pub = rospy.Publisher('localization', Map, queue_size=10)

    def publish(msg):
        self._pub.publish(msg)
