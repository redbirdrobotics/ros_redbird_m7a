from random import randint
from math import pow, sqrt, tan
from threading import Thread

class Ground_Robot_Interface(object):
    """description of class"""
    global iterations
    iterations = 1 / 60

    def __init__(self, x, y, id, color):
        self._x = x
        self._y = y
        self._id = id
        self._color = color

        #radius of ground_robots
        self._radius = 0.34 #m

        #for obstacle robots travelling in a circle
        self._omega = 0

        #generating random velocities
        self._deltaX = ( randint(-33.0, 33.0) ) / (100.0)

        self._deltaY = sqrt( ( (pow(0.33, 2)) - ( pow(self._deltaX, 2) ) ) )

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