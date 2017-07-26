#!/usr/bin/python
import rospy
import time
from redbird_m7a_msgs.msg import *
from simulation_loc import *
 
class Simulation_Node:
    def __init__(self):

        # Initialize node
        rospy.init_node('simulation_node', anonymous=True)

        # Create map listener
        self.red_loc_sub = rospy.Subscriber('localization', RedRobotMap, self.update_red_robot_map)
        self.green_loc_sub = rospy.Subscriber('localization', GreenRobotMap, self.update_green_robot_map)
        #self.obstacle_loc_sub = rospy.Subscriber('localization', ObstacleRobotMap, self.update_obstacle_robot_map)

        # Create publisher
        self._red_pub = rospy.Publisher('/redbird/simulation/robots/red', RedRobotMap, queue_size=1)
        self._green_pub = rospy.Publisher('/redbird/simulation/robots/green', GreenRobotMap, queue_size=1)
        #self._obstacle_pub = rospy.Publisher('/redbird/simulation/robots/obstacle', ObstacleRobotMap, queue_size=1)

        #create service
        self._getting_coordinates_srv = rospy.service('/redbird/simulation/get_future_coords', GetFutureCoords, self.get_future_coords_handler)

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

        # Creating maps
        self.redrobotmap = RedRobotMap()
        self.greenrobotmap = GreenRobotMap()
        #self.obstaclerobotmap = ObstacleRobotMap()
        
        # Create target robot list
        self.red_robot_msgs = []
        self.green_robot_msgs = []
        #self.obstacle_robot_msgs = []

    def red_robot_publish(self, length):
        for robot in xrange(length):
            self.red_robot_msgs.append(Ground_Robot_Position())

        for robot_msg in self.red_robot_msgs:
            for sim_robot in self._sim.get_R_Target_robots():
                robot_msg.id = sim_robot._id
                robot_msg.x = sim_robot._x
                robot_msg.y = sim_robot._y
                robot_msg.vec_x = sim_robot._deltaX
                robot_msg.vec_y = sim_robot._deltaY
                robot_msg.color = sim_robot._color

                break

        self.redrobotmap.ground_robots = self.red_robot_msgs

        self._red_pub.publish(self.redrobotmap)

    def green_robot_publish(self, length):
        
        for robot in xrange(length):
            self.green_robot_msgs.append(Ground_Robot_Position())

        for robot_msg in self.green_robot_msgs:
            for sim_robot in self._sim.get_G_Target_robots():
                robot_msg.id = sim_robot._id
                robot_msg.x = sim_robot._x
                robot_msg.y = sim_robot._y
                robot_msg.vec_x = sim_robot._deltaX
                robot_msg.vec_y = sim_robot._deltaY
                robot_msg.color = sim_robot._color

                break

        self.greenrobotmap.ground_robots = self.green_robot_msgs

        self._green_pub.publish(self.redrobotmap)
 

    def update_red_robot_map(self, msg):
        # Loop through all robots in the localization topic

        self._sim.set_R_Target_robots(len(msg))

        for lRobot in msg.ground_robots:
            # Loop through all robots in the simulation
            for sRobot in self._sim.get_R_Target_robots():
                # If the ids are the same, check the error
                if sRobot._id == lRobot.id:
                    sRobot.check_error(lrobot.x, lRobot.y, lRobot.vec_x, lRobot.vec_y)

        self.red_robot_publish(len(msg))

    def update_green_robot_map(self, msg):

        self._sim.set_G_Target_robots(len(msg))

        for lRobot in msg.ground_robots:
            for sRobot in self._sim.get_G_Target_robots():
                if sRobot._id == lRobot.id:
                    sRobot.check_error(lrobot.x, lRobot.y, lRobot.vec_x, lRobot.vec_y) 

        self.green_robot_publish(len(msg))

        self._ready = True 

    # def update_obstacle_robot_map(self, msg):

    #     self._sim.set_Obstacle_robots(len(msg))

    #     for lRobot in msg.ground_robots:
    #         for sRobot in self._sim.get_Obstacle_robots():
    #             if sRobot._id == lRobot.id:
    #                 sRobot.check_error(lRobot.x, lRobot.y, lRobot.vec_x, lRobot.vec_y)

    #     self.obstacle_robot_publish(len(msg))

    #     # Data has been updated, set ready flag
    #     

    def get_future_coords_handler(self, req):
        
        if(req.color = 0):
            for robot in self._sim.get_R_Target_robots():
                if(req.id == robot._id):
                    current_pos = robot.get_future_coord(req.time)

        elif(req.color == 1):
            for robot in self._sim.get_G_Target_robots():
                if(req.id == robot._id):
                    current_pos = robot.get_future_coord(req.time)

        else:
            for robot in self._sim.get_Obstacle_robots():
                if(req.id == robot._id):
                    current_pos = robot.get_future_coord(req.time)

        return GetFutureCoords(x = current_pos[0], y = current_pos[1])


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