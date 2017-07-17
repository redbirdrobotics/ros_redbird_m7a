from Ground_RobotInterface import Ground_Robot_Interface, iterations
from Sim_Timer import Sim_Timer
from Obstacle_Robot import Obstacle_Robot
from time import sleep
from threading import Thread
from math import fabs, sqrt, tan

class Target_Robot(Ground_Robot_Interface, object):
    """description of class"""

    def __init__(self, x, y, id, color, timer): 
        self._timer = timer

        return super().__init__(x, y, id, color,)

    def update_posX(self):
        changeX = self._deltaX * iterations
        self._x = changeX + self._x
        return self._x

    def update_posY(self):
        changeY = self._deltaY * iterations
        self._y = changeY + self._y
        return self._y
     
    def update_movement(self):

        while not (self._timer._PAUSED.is_set()):

            while self.deltaTime <= 20:
                self.start_timer = self._timer.get_current_timer()
                self.current_pos = (self._x, self._y , self._deltaX, self._deltaY, self._id)
                print(self.current_pos)

                if(self.collision == True):
                    self._deltaX = self._deltaX * -1

                    self.deltaY = self.deltay * -1 

                    sleep(iterations)

                else:

                    self.update_posX()
                    self.update_posY()

                    sleep(iterations)

                self.deltaTime = self._timer.get_current_timer() - self.start_timer

            if(self.deltaTime == 20):
                self.deltaTIme = 0

                self._timerUp = True

        #super().set_coordinates(self.x, self.y) 

    def check_collisions(self, target_robot):
        min_num = 0
        max_num = len(target_robot)

        #arbitrary definition of target robots to be a list of objects
        target_robot = [Target_Robot]
        
        #Only allowing for the PAUSED flag to be false
        while not self._timer._PAUSED.is_set():

            #looping through the array that is being input to the function
            for robot in target_robot:

                #as long as the two id are not equal
                if not (self._id == robot.get_id(self)):

                    #as long as the boundary flag is raised
                    if(self._boundary):
                        #stopping and deleting the thread
                        self._distanceThread._stop()

                        self._distanceThread._delete()

                        break

                    if (self._x >= 10 and self._y >= 10):
                        self._boundary = True

                    else:
                        dXX = robot._x - self._x
                        dYY = robot._y - self._y

                        dCC = sqrt((pow(dXX, 2) + pow(dYY, 2)))

                        if dCC <= 2*radius :
                            self.button_pushed(robot)

                            self.collision = True

            sleep(iterations)

    def oR_check_collisions(self, obstacle_robots = [Obstacle_Robot]):
        #making the obstacle robots an array of Obstacle Robots (arbitrary definition)

        #
        while not self._timer._PAUSED.is_set():

            #looping through the list of Obstacle Robots
            for oRobot in obstacle_robots:
                
                #finding the distance from center to center
                dXX = fabs(oRobot.get_x() - self._x)
                dYY = fabs((oRobot.get_y()) - self._y)
                dCC = sqrt((pow(dXX, 2)) + pow(dYY, 2))

                #if the distance from center to center is the sum of the radii then find the angle of collision
                if dCC <= 2 * self._radius:

                    #creating two vectors between the x and y positions
                    vector_i = self._x - oRobot.get_x()
                    vector_j = self._y - oRobot.get_y()
                    
                    #determining the vector between the two vectors
                    theta = tan(vector_j / vector_i)

                    #Calculating the angle of velocity vector
                    if(self._deltaX == 0):
                         _theta = 90

                    else:
                        _theta = tan(self._deltaX/ self._deltaY)

                    #finding the relative angle needed for the robot's button to be pushed
                    min_theta = _theta - 70
                    max_theta = _theta + 70

                    if(theta >= min_theta and theta <= max_theta):
                        self.button_pushed = True

    def button_pushed(self, robot):
        robot = Target_Robot
         
        vector_i = self._x - robot._x
        vector_j = self._y - robot._y

        theta = tan(vector_j / vector_i)

        #Calculating the angle of velocity vector
        if(self._deltaX == 0):
             _theta = 90

        else:
            _theta = tan(self._deltaX/ self._deltaY)

        min_theta = _theta - 70
        max_theta = _theta + 70

        if(theta >= min_theta and theta <= max_theta):
            self.button_pushed = True

    def run(self, target_robots, obstacle_robots):
        self._distanceThread = Thread(target = self.update_movement)

        self._collision_thread = Thread(target = self.check_collisions, args = (target_robots,))

        self._or_collision_thread = Thread(target = self.oR_check_collisions, args = (obstacle_robots,))

        start_time = self._timer.get_current_timer()
        current_time = 0

        try:
            self._distanceThread.start()

            print("Update movement thread started")

        except:

            print("Thread failed!")

        try:

            self._collision_thread.start()

            print("Collision detection started")

        except:

            print("Thread failed!")

        try:
            self._or_collision_thread.start()

            print("Obstacle Robot collision detection started")

        except:
            print("Thread failed!")

        #while not PAUSED == True and current_time < 20:
        #    self.update_movement()

        #    current_time = self.Timer.get_current_timer() - start_time

        #    sleep(1.0)
    
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
        return super().get_id()

    def get_x(self):
        return super().get_x()

    def get_y(self):
        return super().get_y()