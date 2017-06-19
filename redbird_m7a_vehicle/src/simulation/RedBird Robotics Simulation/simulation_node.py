#!/usr/bin/python

import rospy
import time
from redbird_m7a_msgs.msg import Map, GroundRobotPosition
from Simulation import Simulation
from Target_Robot import Target_Robot

def main():
    # Initialize node
    rospy.init_node('simulation_node', anonymous=True)

    # Create publisher
    pub = rospy.Publisher('simulation', Map, queue_size=10)

    listenerMap = rospy.Subscriber('localization_node', Map, get_Map_data)

    pub1 = rospy.Publisher('simulation', GroundRobotPosition, queue_size=10)

    # Test log
    rospy.loginfo("Simulation started!")

    while not rospy.is_shutdown():
        rospy.loginfo(Simulation.target_robot)
        rospy.loginfo(Simulation.obstacle_robot)
        time.sleep(1)

def get_Map_data(Map):
    for lRobot in Map:
        for sRobot in Simulation.target_robot:
            pass
        pass

if __name__ == '__main__':
    try:
        # Start main
        main()
    except rospy.ROSInterruptException:
        pass
