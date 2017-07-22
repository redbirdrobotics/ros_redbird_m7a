#!/usr/bin/python
import rospy
import time
from redbird_m7a_msgs.msg import Map, GroundRobotPosition
from simulation_2 import *

class Simulation_Node:
    def __init__(self):

        # Initialize node
        rospy.init_node('simulation_node', anonymous=True)

        # Create publisher
        self._pub = rospy.Publisher('simulation', Map, queue_size=10)

        # Create simulation object
        self._sim = Simulation()

        # Start simulation
        self._sim.run()

        # Log start
        rospy.loginfo("Simulation started!")

    def publish_data(self):
        # Create map
        robots = Map()

        # Create target robot list
        robot_msgs = []

        # Create robot messages
        for i in xrange(3):
            robot_msgs.append(GroundRobotPosition())

        # Enter main loop while ROS is running
        while not rospy.is_shutdown():
            # Loop through all simulated robots
            for sim_robot in sim.get_G_Target_robots():
                # Loop through all robot messages that need to be populated
                for robot_msg in robot_msgs:
                    # If the robot message id matches the simulated robot id, update data
                    if robot_msg.id == sim_robot._id:
                        robot_msg.x = sim_robot._x
                        robot_msg.y = sim_robot._y
                        robot_msg.vec_x = sim_robot._deltaX
                        robot_msg.vec_y = sim_robot._deltaY
                        robot_msg.color = sim_robot._color

            # Loop through all simulated robots
            for sim_robot in sim.get_R_Target_robots():
                # Loop through all robot messages that need to be populated
                for robot_msg in robot_msgs:
                    # If the robot message id matches the simulated robot id, update data
                    if robot_msg.id == sim_robot._id:
                        robot_msg.x = sim_robot._x
                        robot_msg.y = sim_robot._y
                        robot_msg.vec_x = sim_robot._deltaX
                        robot_msg.vec_y = sim_robot._deltaY
                        robot_msg.color = sim_robot._color

            #looping through the obstacle robots
            for sim_robot in sim.get_Obtacle_Robots():
                #loop through rest of messages that need to be populated
                for robot_msg in robot_msgs:
                    #if the id matches then populate the message
                    if robot_msg.id == sim_robot._id:
                        robot_msg.x = sim_robot._x
                        robot_msg.y = sim_robot._y
                        robot_msg.vec_x = sim_robot._deltaX
                        robot_msg.vec_y = sim_robot._deltaY
                        robot_msg.color = sim_robot._color

            # Add robots to map
            robots.ground_robots = robot_msgs

            # Publish the map
            self._pub.publish(robots)

if __name__ == '__main__':
    try:
        # Create simulation node object
        sim_n = Simulation_Node()

        rate = rospy.Rate(50)

        # Start simulation data publisher
        while not rospy.is_shutdown():
            sim_n.publish_data()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
