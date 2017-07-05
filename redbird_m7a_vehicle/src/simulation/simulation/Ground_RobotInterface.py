from threading import Thread
from random import randint
from math import tan, pow, sqrt
from Sim_Timer import Sim_Timer, PAUSED
from time import sleep

class Ground_Robot_Interface(Thread, object):
    """description of class"""
    global iterations
    iterations = 1 / 60

    def __init__(self, x, y, id, color):
        self._x = x
        self._y = y
        self._id = id
        self.color = color

        self._deltaX = ((randint(-33, 33)) / 100 )

        self.deltaY = sqrt(((pow(0.33, 2)) - (pow(self._deltaX, 2))))
        
        if(self._deltaX != 0):
            self.angle = tan((self.deltaY / self._deltaX))
        else:
            self.angle = 90

        self.thread_cancelled = False
        self.collision = False

        self.pError = 0.0 
        self.pErrorV = 0.0
        
    def get_coordinates(self):
        return (self.x, self.y)

    def get_id(self):
        return self.ID 

    def set_coordinates(self, x, y):
        self.x = x
        self.y = y

    def get_angle(self):
        pass

    def run(self):
        pass


    def cancel(self):
        pass

    def update_posX(self):
        changeX = self._deltaX * iterations
        self.x = changeX + self._x
        return self._x

    def update_posY(self):
        changeY = self.deltaY * iterations
        self.y = changeY + self._y
        return self._y


    def update_movement(self):
        self.current_pos = (self._x, self._y , self._id)
        print(self.current_pos)

        if(self.collision == True):
            self.deltaX = self._deltaX * -1

            self.deltaY = self.deltay * -1 

            sleep(1)

        else:
            self.update_posX()
            self.update_posY()

    def check_collisions(self):
        pass