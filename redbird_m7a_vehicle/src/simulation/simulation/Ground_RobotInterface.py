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
        self._omega = 0

        self._deltaX = ((randint(-33, 33)) / 100 )

        self._deltaY = sqrt(((pow(0.33, 2)) - (pow(self._deltaX, 2))))

        self.thread_cancelled = False
        self.collision = False
        self._boundary = False
        
    def get_coordinates(self):
        return (self.x, self.y)

    def get_id(self):
        return self.ID 

    def set_coordinates(self, x, y):
        self._x = x
        self._y = y

    def get_angle(self):
        pass

    def run(self):
        pass


    def cancel(self):
        pass

    def update_posX(self):
        changeX = self._deltaX * iterations
        self._x = changeX + self._x
        return self._x

    def update_posY(self):
        changeY = self._deltaY * iterations
        self._y = changeY + self._y
        return self._y

    def update_movement(self):
        self.current_pos = (self._x, self._y , self._id)
        print(self.current_pos)

        if(self.collision == True):
            self.deltaX = self._deltaX * -1

            self._deltaY = self.deltay * -1 

            sleep(1)

        else:
            self.update_posX()
            self.update_posY()

    def check_collisions(self):
        pass