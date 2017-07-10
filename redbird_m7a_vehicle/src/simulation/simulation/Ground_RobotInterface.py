from threading import Thread
from random import randint
from math import tan, pow, sqrt
from Sim_Timer import Sim_Timer
from time import sleep

class Ground_Robot_Interface(Thread, object):
    """description of class"""
    global iterations
    iterations = 1

    def __init__(self, x, y, id, color):
        self._x = x
        self._y = y
        self._id = id
        self.color = color
        self._radius = 1.0 #m

        #for obstacle robots travelling in a circle
        self._omega = 0

        self._deltaX = ((randint(-33, 33)) / 100 )

        self._deltaY = sqrt(((pow(0.33, 2)) - (pow(self._deltaX, 2))))

        self.thread_cancelled = False
        self.collision = False
        self._boundary = False
        
    def update_posX(self):
        pass

    def update_posY(self):
        pass
     
    def update_movement(self):
        pass

    def check_collisions(self, target_robot, obstacle_robots):
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