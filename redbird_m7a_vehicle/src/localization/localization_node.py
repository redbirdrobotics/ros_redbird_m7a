#!/usr/bin/python

import rospy
from redbird_m7a_msgs.msg import Map, GroundRobotPosition

def main():
    # Initialize node
    rospy.init_node('localization_node', anonymous=True)

    # Create publisher
    pub = rospy.Publisher('localization', Map, queue_size=10)

    # Test log
    rospy.loginfo("Localization started!")

if __name__ == '__main__':
    try:
        # Start main
        main()
    except rospy.ROSInterruptException:
        pass
