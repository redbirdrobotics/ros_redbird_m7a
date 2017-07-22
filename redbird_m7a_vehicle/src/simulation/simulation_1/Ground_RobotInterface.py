from random import randint
from math import pow, sqrt, tan
from threading import Thread 

class Ground_Robot_Interface(object):
    """description of class"""
    global iterations
    iterations = 1 / 60

    def __init__(self, x, y, delta_x, delta_y, id, color):
        self._x = x
        self._y = y
        self._id = id
        self._color = color

        #radius of ground_robots
        self._radius = 0.34 #m

        #for obstacle robots travelling in a circle
        self._omega = 0

        #generating random velocities
        self._deltaX = delta_x

        self._deltaY = delta_y

        #creating the flags for the timing, collisions, and border detection respectively
        self._timerUp = False
        self.collision = False
        self._boundary = False

        #creating variables for the timing
        self.start_timer = 0
        self.end_timer = 0
        self.deltaTime = 0
        
    def update_posX(self):
        pass

    def update_posY(self):
        pass
     
    def update_movement(self):
        pass

    def button_pushed(self, robot):
        pass

    def run(self, target_robots, obstacle_robots):
        pass
    
    def change_X_data(self, x):
        pass

    def change_Y_data(self, y):
        pass

    def change_VX_data(self, velocityX):
        pass

    def change_VY_data(self, velocityY):
        pass

    def check_error(self, x, y, velocityX, velocityY):
        pass

    def get_theta(self):

        if(self._deltaX == 0):
            if(self._deltaY < 0):
                theta = 270

            theta = 90
        else:
            theta = tan(self._deltaY / self._deltaX)

        return theta

    def new_direction(self):
        #gets new direction when the timer flag is up
        self._deltaX = randint(-33.0, 33.0) / 100.0

        self._deltaY = sqrt( pow(0.33, 2) - pow(self._deltaX, 2))

    #def check_collisions(self, ground_robots):
    #    #defining an arbitrary array of ground_robots 
    #    ground_robots = [Ground_Robot_Interface]

    #    while not self._timer._PAUSED.is_set():
    #        #looping through the array that is being input to the function
    #        for robot in ground_robots:

    #            #as long as the two colors are equal
    #            if (self._color == robot._color):

    #                #then the id's can't equal
    #                if not(self._id == self._id):

    #                    self.check_calculations(robot)

    #            #if not then
    #            else:
    #                self.check_calculations(robot)

    #def check_calculations(self, robot):
    #    #defining the robot as an arbitrary value
    #    #this method will make it so target and obstacle 
    #    #robots can use this method

    #    robot = Ground_Robot_Interface

    #    #as long as the boundary flag is raised
    #    if(self._boundary):

    #        #stopping and deleting the thread
    #        self._distanceThread._stop()

    #        self._distanceThread._delete()

    #    #testing the if the robot has exited the boundary
    #    elif(self._x >= 10 and self._y >= 10):
    #        self._boundary = True

    #    else:
    #        #Determining the distance from robot to robot
    #        dXX = robot._x - self._x
    #        dYY = robot._y - self._y

    #        dCC = sqrt((pow(dXX, 2) + pow(dYY, 2)))

    #        max_distance = sqrt(self._radius + robot._radius)

    #        #if the distance from center to center is less than the sum of the two radii
    #        if dCC <= max_distance:

    #            self.button_pushed(robot)

    #def button_pushed(self, robot):

    #    #defining arbitrary values for the function input
    #    robot = Ground_Robot_Interface

    #    #finding the distance vectors between each of the robots
    #    vector_i = robot._x - self._x
    #    vector_j = robot._y - self._y

    #    if (vector_i == 0):
    #        theta = 90
    #    else:
    #        theta = tan(vector_j / vector_i)

    #    #Finding the relative angle of where the button is pushed
    #    min_theta = robot.get_theta() - 70
    #    max_theta = robot.get_theta() + 70

    #    if(theta >= min_theta and theta <= max_theta):
    #        self.collision = True

    #def run(self, ground_robots):
    #    self.collision_threading = Thread(target = self.check_collision, args = (ground_robots,))

    #    try:
    #        self.collision_threading.start()

    #        print("Thread started!")

    #    except:
    #        print("Failed")
