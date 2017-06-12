#!/usr/bin/python

import rospy
import time
from redbird_m7a_msgs.msg import Map, GroundRobotPosition

def main():
    # Initialize node
    rospy.init_node('simulation_node', anonymous=True)

    # Create publisher
    pub = rospy.Publisher('simulation', Map, queue_size=10)

    # Test log
    rospy.loginfo("Simulation started!")

    while not rospy.is_shutdown():
        rospy.loginfo("Simulation says hi!")
        time.sleep(1)

if __name__ == '__main__':
    try:
        # Start main
        main()
    except rospy.ROSInterruptException:
        pass
