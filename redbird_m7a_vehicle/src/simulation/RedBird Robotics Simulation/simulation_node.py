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

    map = Map() 
    target_robot = GroundRobotPosition()

    # Test log
    rospy.loginfo("Simulation started!")

    while not rospy.is_shutdown():
        
        for robot in Simulation.target_robot:
            target_robot.x = robot.x

            target_robot.y = robot.y

            target_robot.id = robot.id

            target_robot.vec_x = robot.deltaX

            target_robot.vec_y = robot.deltaY

            target_robot.color = robot.color

            map.data = target_robot

            pub.publish(map)

        time.sleep(1)

def get_Map_data(Map):
    for lRobot in Map:
        for sRobot in Simulation.target_robot:
            while(sRobot.id == lRobot.id):
                sRobot.check
        pass

if __name__ == '__main__':
    try:
        # Start main
        main()
    except rospy.ROSInterruptException:
        pass
