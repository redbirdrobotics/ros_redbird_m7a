#!/usr/bin/python

import rospy
import cv2
import numpy as np
import time
from redbird import Camera, Robot, Utilities
from redbird_m7a_msgs.msg import Map, GroundRobotPosition

def main():
    # Initialize node
    rospy.init_node('localization_node', anonymous=True)

    # Create publisher
    pub = rospy.Publisher('localization', Map, queue_size=10)

    target_robot1 = GroundRobotPosition()
    target_robot2 = GroundRobotPosition()
    target_robot3 = GroundRobotPosition()
    target_robot4 = GroundRobotPosition()
    target_robot5 = GroundRobotPosition()
    target_robot6 = GroundRobotPosition()
    target_robot7 = GroundRobotPosition()
    target_robot8 = GroundRobotPosition()
    target_robot9 = GroundRobotPosition()
    target_robot10 = GroundRobotPosition()

    obstacle_robot1 = GroundRobotPosition()
    obstacle_robot2 = GroundRobotPosition()
    obstacle_robot3 = GroundRobotPosition()
    obstacle_robot4 = GroundRobotPosition()

    target_robots = [target_robot1, target_robot2, target_robot3, target_robot4, target_robot5, target_robot6, target_robot7, target_robot8, target_robot9, target_robot10]
    obstacle_robots = [obstacle_robot1, obstacle_robot2, obstacle_robot3, obstacle_robot4]

    target_robot1.id = 
    target_robot1.type = 
    target_robot1.x = 
    target_robot1.y = 
    target_robot1.vec_x = 
    target_robot1.vec_y = 
    target_robot1.color =
    target_robot1.confidence = 
    target_robot1.out_of_bounds =   



    mapp = Map()
    mapp.target_robots = target_robots
    mapp.obstacle_robots = obstacle_robots


    pub.publish(mapp)

    # Test log
    rospy.loginfo("Localization started!")

if __name__ == '__main__':
    try:
        # Start main
        main()
    except rospy.ROSInterruptException:
        pass
