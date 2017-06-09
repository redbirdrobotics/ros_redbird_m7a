#!/usr/bin/python

import rospy
from redbird_m7a.msg import Map, GroundRobotPosition

def main():
    # Create publisher
    pub = rospy.Publisher('localization', Map, queue_size=10)

    # Initialize publisher node
    rospy.init_node('localization_node', anonymous=True)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
