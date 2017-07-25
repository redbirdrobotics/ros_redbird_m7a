#!/usr/bin/python
import rospy
import time
from redbird_m7a_msgs.msg import *
from simulation_1 import *
 
class Simulation_Node:
    def __init__(self):

         # Initialize node
        rospy.init_node('simulation_node', anonymous=True)

        # Create map listener
        self._loc_sub = rospy.Subscriber('localization', RedRobotMap, self.update_red_robot_map)
        self._loc_sub = rospy.Subscriber('localization', GreenRobotMap, self.update_green_robot_map)
        self._loc_sub = rospy.Subscriber('localization', ObstacleRobotMap, self.update_obstacle_robot_map)

        # Create publisher
        self._red_pub = rospy.Publisher('/redbird/simulation/robots/red', RedRobotMap, queue_size=1)
        self._green_pub = rospy.Publisher('/redbird/simulation/robots/green', GreenRobotMap, queue_size=1)
        self._obstacle_pub = rospy.Publisher('/redbird/simulation/robots/obstacle', ObstacleRobotMap, queue_size=1)

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
        # Creating maps
        redrobotmap = RedRobotMap()
        greenrobotmap = GreenRobotMap()
        obstaclerobotmap = ObstacleRobotMap()
        
        # Create target robot list
        red_robot_msgs = []
        green_robot_msgs = []
        obstacle_robot_msgs = []
         
        for robot in xrange(5):
            red_robot_msgs.append(Ground_Robot_Position())
            green_robot_msgs.append(Ground_Robot_Position())

        for robot in xrange(4):
            obstacle_robot_msgs.append(Ground_Robot_Position())

        #looping through target_robots
        for sim_robot in sim.get_Red_Target_Robots():
            #loop through the messages need to be populated
            for robot_msg in red_robot_msgs:
                robot_msg.id = sim_robot._id
                robot_msg.x = sim_robot._x
                robot_msg.y = sim_robot._y
                robot_msg.vec_x = sim_robot._deltaX
                robot_msg.vec_y = sim_robot._deltaY
                robot_msg.color = sim_robot._color

        for sim_robot in sim.get_Green_Target_Robots():
            for robot_msg in green_robot_msgs:
                robot_msg.id = sim_robot._id
                robot_msg.x = sim_robot._x
                robot_msg.y = sim_robot._y
                robot_msg.vec_x = sim_robot._deltaX
                robot_msg.vec_y = sim_robot._deltaY
                robot_msg.color = sim_robot._color

        #looping through the obstacle robots
        for sim_robot in sim.get_Obtacle_Robots():
            #loop through rest of messages that need to be populated
            for robot_msg in obstacle_robot_msgs:
                robot_msg.id = sim_robot._id
                robot_msg.x = sim_robot._x
                robot_msg.y = sim_robot._y
                robot_msg.vec_x = sim_robot._deltaX
                robot_msg.vec_y = sim_robot._deltaY
                robot_msg.color = sim_robot._color
            
        # Add robots to map
        redrobotmap.ground_robots = red_robot_msgs
        greenrobotmap.ground_robots = green_robot_msgs
        obstaclerobotmap.ground_robots = obstacle_robot_msgs

        # Publish the map
        self._red_pub.publish(redrobotmap)
        self._green_pub.publish(greenrobotmap)
        self._obstacle_pub.publish(obstaclerobotmap)

    def update_red_robot_map(self, msg):
        # Loop through all robots in the localization topic
        for lRobot in msg.ground_robots:
            # Loop through all robots in the simulation
            for sRobot in self._sim.get_R_Target_robots():
                # If the ids are the same, check the error
                if sRobot._id == lRobot.id:
                    sRobot.check_error(lrobot.x, lRobot.y, lRobot.vec_x, lRobot.vec_y)

    def update_green_robot_map(self, msg):
        for lRobot in msg.ground_robots:
            for sRobot in self._sim.get_G_Target_robots():
                if sRobot._id == lRobot.id:
                    sRobot.check_error(lrobot.x, lRobot.y, lRobot.vec_x, lRobot.vec_y)

    def update_obstacle_robot_map(self, msg):
        for lRobot in msg.ground_robots:
            for sRobot in self._sim.get_Obstacle_Robots():
                if sRobot._id == lRobot.id:
                    sRobot.check_error(lRobot.x, lRobot.y, lRobot.vec_x, lRobot.vec_y)

        # Data has been updated, set ready flag
        self._ready = True


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