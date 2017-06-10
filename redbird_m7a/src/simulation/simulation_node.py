#!/usr/bin/python

import rospy
import time
from redbird_m7a.msg import Map, GroundRobotPosition

def main():
    # Create publisher
    pub = rospy.Publisher('simulation', Map, queue_size=10)

    # Initialize node
    rospy.init_node('simulation_node', anonymous=True)

    # Test log
    rospy.loginfo("Simulation started!")

    while not rospy.is_shutdown():
        rospy.loginfo("Simulation says hi!")
        time.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
