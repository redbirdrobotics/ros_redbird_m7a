#!/usr/bin/python

# TODO: Add header docstring

import rospy
import time
import flightsys
from flightsys import Controller, Control_Mode
from utils import Vehicle, Flight_Controller, Flight_Mode
from redbird_m7a_msgs.msg import Map, GroundRobotPosition


class Competition(flightsys.Flight, object):
    def __init__(self, vehicle):
        # Call super constructor
        super(Competition, self).__init__(name='comp_flight', log_tag='[COMP FLIGHT]', vehicle=vehicle)

        # Initialize variables
        self._loc_map = Map()
        self._sim_map = Map()
        self._firstTime = True
        self._numGroundRobots = 10
        self._priorityRobot = None
        self._attempts = 0
        self._successful = False
        self._centerX = 0
        self._centerY = 0
        self._landX = 0
        self._landY = 0
        self._minDroneToRobotDistance = 0.5
        self._lastLandSuccesful = False
        self._startTime = time.time()

        # Initialize vehicle for tracking
        self._vehicle = Vehicle()

        # Initialize flight controller
        #??? self._flight_controller = Flight_Controller()

        # Create subscribers
        self._loc_sub = rospy.Subscriber("localization", Map, self.update_loc_map)
        self._sim_sub = rospy.Subscriber("simulation", Map, self.update_sim_map)
        
    def getFlightTag(self):
        return "[flight_node] "

    def update_loc_map(self, data):
        self._loc_map = data

    def update_sim_map(self, data):
        self._sim_map = data

    def getPriorityRobotByConfidence(self):
        tempConfidence = -1.0
        tempPriority = None
        for robot in self._sim_map.target_robots:
            if (robot.confidence > tempConfidence and robot.out_of_bounds == False):
                tempPriority = robot
                tempConfidence = robot.confidence
        return tempPriority

    def getNumRobotsInBounds(self):
        count = 0
        for robot in self._sim_map.target_robots:
            if (robot.out_of_bounds == False):
                count+=1
        return count


    def getDistanceToGroundRobot(self, robot):
        x = robot.x - self._vehicle.x
        y = robot.y - self._vehicle.y
        z = x**2 + y**2
        z = z**(1/2.0)
        return z

    def priorityRobotOrientedCorrectly(self):
        print("placeholder")

    def getDroneNearPriorityRobot(self, x):
        while (self.distanceToGroundRobot(self._priorityRobot) >= x):

            # Fly to point (priority robot)
            target_point = (self._priorityRobot.x, self._priorityRobot.y, 2.5)
            rospy.loginfo(self._log_tag + "Flying to %s" % (target_point,))
            self._c.set_position(target_point)
            self._c.set_mode(Control_Mode.POSITION)
        
            # Wait for position to be reached
            while not rospy.is_shutdown() and self._c.get_mode() == Control_Mode.POSITION:
                pass

    def flyToGround(self, x, y):
        print("placeholder")

    def orientGroundRobot(self, robot):
        print("placeholder")

    def landOnGroundRobot(self, x, y):
        print("need a complex landing function here")
        self._lastLandSuccesful = True

    #Make a function that constantly updates the ground robot array because robots leave bounds

    def start(self):
        try:
            if not rospy.is_shutdown():

                #Initialize robot if first run
                if (self._firstTime == True):
                    
                    # Arm the vehicle
                    self._v.arm()
                    rospy.loginfo(self._log_tag + "Vehicle armed")
                    
                    # Wait a moment
                    rospy.sleep(2)
                    
                    # Set takeoff altitude
                    self._c.set_takeoff_altitude(2.5)
                    print self._c.get_takeoff_altitude()
                    
                    # Switch mode to takeoff
                    self._c.set_mode(Control_Mode.TAKEOFF)
                    print self._c.get_mode()
                    
                    # Wait for takeoff to complete
                    while not rospy.is_shutdown() and self._c.get_mode() == Control_Mode.TAKEOFF:
                        pass
                    
                    # Fly to point (center)
                    target_point = (self._centerX, self._centerY, 2.5)
                    rospy.loginfo(self._log_tag + "Flying to %s" % (target_point,))
                    self._c.set_position(target_point)
                    self._c.set_mode(Control_Mode.POSITION)
        
                    # Wait for position to be reached
                    while not rospy.is_shutdown() and self._c.get_mode() == Control_Mode.POSITION:
                        pass
                    
                    #Finish startup
                    rospy.loginfo(self.getFlightTag(self) + "Drone has reached the center.")
                    self._firstTime = False
    
                #Complete Loop
                while (self.getNumRobotsInBounds() > 0):
                    
                    #Allocate the next priority ground robot, currently finding priority by simulation confidence
                    self._priorityRobot = self.getPriorityRobotByConfidence()
    
                    #Fly drone near robot for the first time if still in bounds
                    if (self._priorityRobot.out_of_bounds == False): 
                        self.getDroneNearPriorityRobot(self._minDroneToRobotDistance)
    
                    #Check to see if ground robot has crossed the goal line or gone out of bounds
                    while (self._priorityRobot.out_of_bounds == False):
                        
                        #Check to see if ground robot is oriented towards goal line
                        if (self.priorityRobotOrientedCorrectly() == False):
                            
                            #Land on the robot and rotate it
                            self.landOnGroundRobot(self._priorityRobot.x, self._priorityRobot.y)
                            self.orientGroundRobot(self._priorityRobot)
                            
                            # Fly back to 2.5 meters high, Switch mode to takeoff
                            self._c.set_mode(Control_Mode.TAKEOFF)
                            print self._c.get_mode()
                    
                            # Wait for takeoff to complete
                            while not rospy.is_shutdown() and self._c.get_mode() == Control_Mode.TAKEOFF:
                                pass
                            
                            #Add an attempt every time you must rotate the robot
                            self._attempts+=1
                            
                            #If the drone has taken too long on a certain robot
                            if (self._attempts >= 5):
                                
                                #Reset attempts and fetch a new priority robot
                                self._attempts = 0
                                self._priorityRobot = self.getPriorityRobotByConfidence()
    
                        #Fly drone near robot again
                        self.getDroneNearPriorityRobot(self._minDroneToRobotDistance)
    
                    #End of while loop, a robot has crossed goal line or gone out of bounds
    
                    self._attempts = 0
    
                #Done!
                rospy.loginfo(self.getFlightTag() + "There are no more ground robots left.")
                
                #Fly to point (land)
                target_point = (self._landX, self._landY, 2.5)
                rospy.loginfo(self._log_tag + "Flying to %s" % (target_point,))
                self._c.set_position(target_point)
                self._c.set_mode(Control_Mode.POSITION)
        
                # Wait for position to be reached
                while not rospy.is_shutdown() and self._c.get_mode() == Control_Mode.POSITION:
                    pass
                
                #switch mode to land
                rospy.loginfo(self._log_tag + "Landing")
                self._c.set_mode(Control_Mode.LAND)
    
                # Wait for land to complete
                while not rospy.is_shutdown() and self._c.get_mode() == Control_Mode.LAND:
                    pass
                
                #Complete
                rospy.loginfo(self.getFlightTag() + "Successfully Landed.")
                return
            
        except KeyboardInterrupt:
            pass

