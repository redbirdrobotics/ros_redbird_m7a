#!/usr/bin/python
import rospy
import time
from redbird_m7a_msgs.msg import Map, GroundRobotPosition
from simulation import *

class Simulation_Node:
    def __init__(self):
         # Initialize node
        rospy.init_node('simulation_node', anonymous=True)

        # Create publisher
        self._pub = rospy.Publisher('simulation', Map, queue_size=10)

        # Create map listener
        self._loc_sub = rospy.Subscriber('localization', Map, self.update_map)

        #creating listener for when the timer is up on target robots
        self._timing_sub = rospy.Subscriber('localization', Map, self.get_data_for_robots)

        # Create simulation object 
        self._sim = Simulation()

        # Log waiting for ready flag
        rospy.loginfo("Waiting for initial information from localization...")

        # Wait for data to be populated from localization info
        self._ready = False
        while not self._ready:
           pass
       
        # Start simulation
        self._sim.run()

        # Log start
        rospy.loginfo("Simulation started!")

    def publish_data(self):
        # Create map
        map = Map() 
    
        # Create target robot list
        robot_msgs = []

        # Create all GroundRobotPositions
        for x in xrange(14):
            target_robots.append(GroundRobotPosition())

        # Enter main loop while ROS is running
        while not rospy.is_shutdown():
            # Loop through all simulated robots
            for sim_robot in sim.get_Target_robots():
                # Loop through all robot messages that need to be populated
                for robot_msg in robot_msgs: 
                    # If the robot message id matches the simulated robot id, update data
                    if robot_msg.id == sim_robot.get_id():
                        robot_msg.x = sim_robot.x
                        robot_msg.y = sim_robot.y
                        robot_msg.vec_x = sim_robot.deltaX
                        robot_msg.vec_y = sim_robot.deltaY
                        robot_msg.color = sim_robot.color
            
            #looping through the obstacle robots
            for sim_robot in sim.get_Obtacle_Robots():
                #loop through rest of messages that need to be populated
                for robot_msg in robot_msgs:
                    #if the matches then populate the message
                    if robot_msg.id == sim_robot.get_id():
                        robot_msg.x = sim_robot.x
                        robot_msg.x = sim_robot.x
                        robot_msg.y = sim_robot.y
                        robot_msg.vec_x = sim_robot.deltaX
                        robot_msg.vec_y = sim_robot.deltaY
                        robot_msg.color = sim_robot.color
            
            # Add robots to map
            map.ground_robots = robot_msgs

            # Publish the map
            self._pub.publish(map)

    def update_map(self, msg):
        # Loop through all robots in the localization topic
        for lRobot in msg.ground_robots:
            # Loop through all robots in the simulation
            for sRobot in self._sim.get_Target_robots():
                # If the ids are the same, check the error
                if sRobot.id == lRobot.id:
                    sRobot.checkError(lrobot.x, lRobot.y, lRobot.vec_x, lRobot.vec_y)

        # Data has been updated, set ready flag
        self._ready = True
        while not rospy.is_shutdown():
            rospy.loginfo("Simulation says hi!")
            time.sleep(10)

    def get_data_for_robots(self):
        #looping through simulated robots
        for tRobot in self._sim.get_Target_robots():
            #advancing only if the timerUp flag is true (which is raised when the 20s is up)
            while tRobot.timerUp:
                #looping through localization robots
                for lRobot in msg.ground_robots:
                    #if the id matches update all data
                    if (tRobot.id == lRobot.id):
                        tRobot.update_data(lRobot.x, lRobot.y, lRobot.vec_x, vec_y)


if __name__ == '__main__':
    try:
        # Create simulation node object
        sim_n = Simulation_Node()
        
        # Start simulation data publisher
        sim_n.publish_data()
    except rospy.ROSInterruptException:
        pass