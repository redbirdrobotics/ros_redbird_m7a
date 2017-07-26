from Ground_RobotInterface import Ground_Robot_Interface, iterations
from Sim_Timer import Sim_Timer
from Obstacle_Robot import Obstacle_Robot
from threading import Thread
from math import fabs, sqrt, tan
import rospy

class Target_Robot(Ground_Robot_Interface):
    """description of class"""

    def __init__(self, x, y, deltax, deltay, id, color, timer): 
        self._timer = timer

        return super(Target_Robot, self).__init__(x, y, deltax, deltay, id, color)
        
    def update_posX(self):
        changeX = self._deltaX * iterations
        self._x = changeX + self._x
        return self._x

    def update_posY(self):
        changeY = self._deltaY * iterations
        self._y = changeY + self._y
        return self._y
     
    def update_movement(self):

        while not rospy.is_shutdown():
            #adds the ability to pause and resume timer
            while not (self._timer._quit.is_set()):

                #while the paused flag is not raised
                while not (self._timer._PAUSED.is_set()):
                    self.start_timer = self._timer.get_current_timer()

                    #as long as the time elapsed is less than 20
                    while self.deltaTime < 20:

                        if(self.collision == True):
                            
                            self._deltaX = self._deltaX * -1

                            self._deltaY = self._deltaY * -1 

                            self.collision = False

                        else:
                            self.update_posX() 

                            self.update_posY()

                        self.deltaTime = self._timer.get_current_timer() - self.start_timer

                        rospy.sleep(iterations)

                    if(self.deltaTime == 20):

                        self.deltaTIme = 0

                        self._timerUp = True

    def oR_check_collisions(self, obstacle_robots = [Obstacle_Robot]):
        #making the obstacle robots an array of Obstacle Robots (arbitrary definition)

        #as long as the timer flag is not set
        while not self._timer._PAUSED.is_set():

            #looping through the list of Obstacle Robots
            for oRobot in obstacle_robots:
                
                #finding the distance from center to center
                dXX = fabs((oRobot.get_x()) - self._x)
                dYY = fabs((oRobot.get_y()) - self._y)
                dCC = sqrt((pow(dXX, 2)) + pow(dYY, 2))

                #creating the max distance the robots can be
                max_distance = (self._radius + oRobot._radius)

                if dCC <= max_distance:

                    #creating two vectors between the x and y positions
                    vector_i = oRobot.get_x() - self._x
                    vector_j = oRobot.get_y() - self._y
                    
                    #determining the vector between the two robots and bounds checking the angle
                    if(vector_i == 0):

                        if(vector_j < 0):

                            theta = 270

                        theta = 90

                    else:

                        theta = tan(vector_j / vector_i)

                    #finding the relative angle needed for the robot's button to be pushed
                    min_theta = self.get_theta() - 70
                    max_theta = self.get_theta() + 70

                    if(theta >= min_theta and theta <= max_theta):
                        self.collision = True

                rospy.sleep(iterations)

    def run(self):
        self._distanceThread = Thread(target = self.update_movement)

        #self._or_collision_thread = Thread(target = self.oR_check_collisions, args = (obstacle_robots,))

        start_time = self._timer.get_current_timer()
        current_time = 0

        try:
            self._distanceThread.start()

        except:

            rospy.loginfo("Thread failed!")

        #try:
        #    self._or_collision_thread.start()

        #    print("Obstacle Robot collision detection started")

        #except:
        #    print("Thread failed!")
    
    def change_X_data(self, x):
        self._x = x

    def change_Y_data(self, y):
        self._y = y

    def change_VX_data(self, velocityX):
        self._deltaX = velocityX

    def change_VY_data(self, velocityY):
        self._deltaY = velocityY

    def check_error(self, x, y, velocityX, velocityY):
        pErrorX = ((self._x - x) /  x)
        
        if not (pErrorX <= 0.001):
            self.change_X_data(x)

        pErrorY = ((self._y - y) /  y) 

        if not (pErrorY <= 0.001):
            self.change_Y_data(y)

        pErrorVX = ((self._deltaX - velocityX) / velocityX)

        if not (pErrorVX <= 0.001):
            self.change_VX_data(velocityX)

        pErrorVY = ((self._deltaY - velocityY) / velocityY)

        if not (pErrorVY<= 0.001):
            self.change_VY_data(velocityY)

    def update_data(self, x, y, velocityX, velocityY):
        self.change_X_data(x)

        self.change_Y_data(y)
        
        self.change_VX_data(velocityX)

        self.change_VY_data(velocityY)

    def get_id(self):
        return self._id

    def get_x(self):
        return self._x

    def get_y(self):
        return self._y

    def get_radius(self):
        return self._radius

    def get_future_coord(self, time):
        deltaX = self._deltaX * time
        deltaY = self._deltaY * time

        current_pos = ((self._x + deltaX), (self._y + deltaY))

        return current_pos